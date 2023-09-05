#!/usr/bin/python
from __future__ import print_function
from future.builtins import input
import sys, tty, termios
import time

import stretch_body.hello_utils as hu
import math
import yaml
import cma
import argparse as ap
import glob
import numpy as np

import open3d as op
from scipy.optimize import minimize, minimize_scalar
import matplotlib.pyplot as plt
import matplotlib
import yaml


import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

def use_all_data(sample_number, sample):
    return True

def use_very_little_data(sample_number, sample):
    if (sample_number % 20) == 0:
        return True
    else:
        return False

    
def angle_diff_rad(target_rad, current_rad):
    # I've written this type of function many times before, and it's
    # always been annoying and tricky. This time, I looked on the web:
    # https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
    diff_rad = target_rad - current_rad
    diff_rad = ((diff_rad + math.pi) % (2.0 * math.pi)) - math.pi
    return diff_rad


def create_time_string():
    t = time.localtime()
    time_string = str(t.tm_year) + str(t.tm_mon).zfill(2) + str(t.tm_mday).zfill(2) + str(t.tm_hour).zfill(2) + str(t.tm_min).zfill(2) + str(t.tm_sec).zfill(2)
    return time_string
    
def axes_from_normal(normal):
    # attempts to uses axes that are close to the axes of the
    # coordinate system
    normal = normal / np.linalg.norm(normal)
    axis_similarity = np.abs(normal.flatten())
    least_similar_axis = np.argmin(axis_similarity)
    x_axis = np.zeros(3, np.float32)
    x_axis[least_similar_axis] = 1.0
    x_axis = np.reshape(x_axis, (3,1))
    x_height = np.dot(normal.transpose(), x_axis)
    x_axis = x_axis - (x_height * normal)
    x_axis = x_axis / np.linalg.norm(x_axis)
    #print('normal =', normal)
    #print('x_axis =', x_axis)
    y_axis = np.cross(normal.flatten(), x_axis.flatten())
    y_axis = y_axis / np.linalg.norm(y_axis)

    normal = np.reshape(normal, (3,1))
    y_axis = np.reshape(y_axis, (3,1))

    return normal, x_axis, y_axis


def circle_of_points(center, normal, radius, num_points=100):
    start = 0.0
    end = 2.0 * np.pi
    delta_angle = (end - start) / num_points
    theta = np.arange(0.0, 2.0 * np.pi, delta_angle)
    n, x_axis, y_axis = axes_from_normal(normal)
    samples = center + ((radius * np.cos(theta)) * x_axis) + ((radius * np.sin(theta)) * y_axis)
    return samples


# ################################################################
def align_vectors(f,t):
   # f and t are in the form of numpy array
   # create rotation matrix that aliggns f to t
   # https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d/897677
   v= np.cross(f, t)
   u = v/np.linalg.norm(v)
   c = np.dot(f, t)
   h = (1 - c)/(1 - c**2)
   vx, vy, vz = v
   rot =[[c + h*vx**2, h*vx*vy - vz, h*vx*vz + vy],
         [h*vx*vy+vz, c+h*vy**2, h*vy*vz-vx],
         [h*vx*vz - vy, h*vy*vz + vx, c+h*vz**2]]
   return np.array(rot)

def write_configuration_params(cma_result):
    import stretch_body.pimu
    pimu = stretch_body.pimu.Pimu()
    params = cma_result['best_parameters']

    A=np.array(params[3:6])
    B = np.array([0.0, 0.0, 1.0])
    R=align_vectors(A,B)
    pimu.write_configuration_param_to_YAML('pimu.config.mag_offsets',[params[0],params[1],params[2]])
    pimu.write_configuration_param_to_YAML('pimu.config.mag_softiron_matrix',[
        float(R[0][0]),
        float(R[0][1]),
        float(R[0][2]),
        float(R[1][0]),
        float(R[1][1]),
        float(R[1][2]),
        float(R[2][0]),
        float(R[2][1]),
        float(R[2][2])])
    pimu.write_configuration_param_to_YAML('pimu.config.gyro_zero_offsets', [0,0,0])
    pimu.write_configuration_param_to_YAML('pimu.config.rate_gyro_vector_scale', cma_result['rate_gyro_scale'])
    pimu.write_configuration_param_to_YAML('pimu.config.gravity_vector_scale', cma_result['gravity_scale'])

# ################################################################

class MobileBaseImuCalibrator:
    def __init__(self, calibration_directory, sample_selector_func, calibration_options, visualize, validation_run,qc):
        self.visualize = visualize
        self.use_this_sample = sample_selector_func 
        self.calibration_directory = calibration_directory
        self.data = None
        self.qc=qc
        self.validation_run=validation_run
        self.num_parameters = len(self.get_names_of_parameters_to_fit())
        self.initial_center, self.initial_normal, self.initial_radius, self.initial_turn_axis, self.initial_gravity = self.unpack_parameters(np.zeros(self.num_parameters))
        
    def load_data(self):
        if self.validation_run:
            filenames = glob.glob(self.calibration_directory + hu.get_fleet_id()+'_base_imu_calibration_data_validate_' + '[0-9]*[0-9].yaml')
        else:
            filenames = glob.glob(self.calibration_directory + hu.get_fleet_id()+'_base_imu_calibration_data_' + '[0-9]*[0-9].yaml')
        filenames.sort()
        most_recent_filename = filenames[-1]
        print('Loading most recent head calibration data from a YAML file named ' + most_recent_filename)
        fid = open(most_recent_filename, 'r')
        self.data = yaml.load(fid,Loader=yaml.FullLoader)
        fid.close()
                    
        #print('loaded data =', self.data)
        self.calculate_normalization()

        self.rate_gyro_data = np.array([[s['imu']['gx'], s['imu']['gy'], s['imu']['gz']]
                                        for i,s in enumerate(self.data)
                                        if self.use_this_sample(i,s)])
        
        self.magnetometer_data = np.array([[s['imu']['mx'], s['imu']['my'], s['imu']['mz']]
                                           for i,s in enumerate(self.data)
                                           if self.use_this_sample(i,s)])
        
        self.accelerometer_data = np.array([[s['imu']['ax'], s['imu']['ay'], s['imu']['az']]
                                            for i,s in enumerate(self.data)
                                            if self.use_this_sample(i,s)])

        self.mobile_base_data =  np.array([[s['base']['theta'], s['base']['theta_vel']]
                                            for i,s in enumerate(self.data)
                                            if self.use_this_sample(i,s)])

        self.time_data =  np.array([s['time']
                                    for i,s in enumerate(self.data)
                                    if self.use_this_sample(i,s)])
        
    def get_calibration_options(self):
        calibration_options = {}
        return calibration_options
            
    def get_names_of_parameters_to_fit(self):
        # A simple circular model of magnetometer measurements from a
        # 360 degree turn of the mobile base.
        parameter_names = []
        magnetometer_parameters = ['magnetometer_center_x', 'magnetometer_center_y', 'magnetometer_center_z',
                                   'magnetometer_normal_x', 'magnetometer_normal_y', 'magnetometer_normal_z',
                                   'magnetometer_radius']
        parameter_names.extend(magnetometer_parameters)

        # Unit scale mobile base rotational axis when turning. This
        # axis should represent the rate gyro measurements for an
        # ideal mobile base turn of 1 rad/s.
        rate_gyro_parameters = ['rate_gyro_unit_turn_axis_x', 'rate_gyro_unit_turn_axis_y', 'rate_gyro_unit_turn_axis_z']
        parameter_names.extend(rate_gyro_parameters)

        # Unit scale mobile base rotational axis when turning. This
        # axis should represent the rate gyro measurements for an
        # ideal mobile base turn of 1 rad/s.
        accelerometer_parameters = ['accelerometer_gravity_x', 'accelerometer_gravity_y', 'accelerometer_gravity_z']
        parameter_names.extend(accelerometer_parameters)
        
        return parameter_names

    
    def initial_parameters(self):
        #print('magnetometer_data.shape =', self.magnetometer_data.shape)
        center = np.mean(self.magnetometer_data, axis=0)
        #print('center.shape =', center.shape)
        #center = np.reshape(center, (3,1))
        #print('center.shape =', center.shape)
        normal = np.array([0.0, 1.0, 0.0])
        from_center = self.magnetometer_data - center.flatten()
        #print('from_center.shape =', from_center.shape)
        #print('from_center =', from_center)
        radius = np.mean(np.linalg.norm(from_center.transpose(), axis=0))
        #print('########################################')
        #print('center =', center)
        #print('radius =', radius)
        #print('self.magnetometer_data =', self.magnetometer_data)
        #print('########################################')

        turn_axis = np.mean(self.rate_gyro_data, axis=0)
        gravity = np.mean(self.accelerometer_data, axis=0)
        
        parameters = self.pack_parameters(center, normal, radius, turn_axis, gravity)
        #exit()
        return parameters

    def set_soft_constraints(self, initial_parameters):
        self.initial_center, self.initial_normal, self.initial_radius, self.initial_turn_axis, self.initial_gravity = self.unpack_parameters(initial_parameters)
    
    def parameter_error_term(self, parameters):
        #print('self.initial_center, self.initial_normal, self.initial_radius = ', self.initial_center, self.initial_normal, self.initial_radius)
        center, normal, radius, turn_axis, gravity = self.unpack_parameters(parameters)
        center_change = np.linalg.norm(center - self.initial_center) / np.linalg.norm(self.initial_center)
        normal_unit_error = np.abs(1.0 - np.linalg.norm(normal))
        normal_angle_change = np.abs(np.dot(normal.transpose(), self.initial_normal) - 1.0)
        radius_change = np.abs(radius - self.initial_radius) / self.initial_radius
        #print('radius_change =', radius_change)

        parameter_error = 0.0
        parameter_error += normal_unit_error * 100.0
        if normal_angle_change > 0.3:
            #parameter_error += (normal_angle_change - 0.3) * 100.0
            pass
        if radius_change > 0.2: #0.5 didn't work
            parameter_error += (radius_change - 0.2) * 100.0
        if center_change > 0.5:
            #parameter_error += (center_change - 0.5) * 100.0
            pass

        #print('parameter_error =', parameter_error)
        return parameter_error

    
    def calculate_normalization(self):
        return

    def unpack_parameters(self, parameters): 
        # magnetometer
        center = parameters[:3]
        center = np.reshape(center, (3,1))
        normal = parameters[3:6]
        normal = np.reshape(normal, (3,1))
        radius = parameters[6]
        # rate gyro
        turn_axis = parameters[7:10]
        turn_axis = np.reshape(turn_axis, (3,1))
        # accelerometer
        gravity = parameters[10:13]
        gravity = np.reshape(gravity, (3,1))
        return center, normal, radius, turn_axis, gravity
    
    def pack_parameters(self, center, normal, radius, turn_axis, gravity):
        # magnetometer
        parameters = np.zeros(self.num_parameters)
        parameters[:3] = center[:]
        parameters[3:6] = normal[:]
        parameters[6] = radius
        # rate gyro
        parameters[7:10] = turn_axis[:]
        # accelerometer
        parameters[10:13] = gravity[:]
        return parameters
    
    def visualize_fit(self, parameters_to_fit):
        center, normal, radius, turn_axis, gravity = self.unpack_parameters(parameters_to_fit)

        torus = op.geometry.TriangleMesh.create_torus(torus_radius=radius, tube_radius=0.2, radial_resolution=30, tubular_resolution=20)
        torus.translate(center)
        n = normal/np.linalg.norm(normal)
        proj = np.dot(np.array([0.0, 0.0, 1.0]), n)
        #print('proj =', proj)
        rad = np.arccos(proj)
        #print('rad =', rad)
        cross = np.cross(np.array([0.0, 0.0, 1.0]).flatten(), n.flatten())
        #print('cross =', cross)
        rotate = rad * (cross/np.linalg.norm(cross))
        #print('rotate =', rotate)
        #torus.rotate(rotate, center=True, type=op.geometry.RotationType.AxisAngle)
        rotation_matrix = op.geometry.get_rotation_matrix_from_axis_angle(rotate)
        #torus.rotate(rotation_matrix, center=True)
        torus.rotate(rotation_matrix)
        
        #circle_points = circle_of_points(center, normal, radius)
        #circle_point_cloud = op.geometry.PointCloud()
        #circle_point_cloud.points = op.utility.Vector3dVector(circle_points.transpose())
        
        magnetometer_point_cloud = op.geometry.PointCloud()
        magnetometer_point_cloud.points = op.utility.Vector3dVector(self.magnetometer_data)
        
        rate_gyro_point_cloud = op.geometry.PointCloud()
        rate_gyro_point_cloud.points = op.utility.Vector3dVector(self.rate_gyro_data)
        rate_gyro_point_cloud.paint_uniform_color([0.0, 0.0, 1.0])
        rate_gyro_sphere = op.geometry.TriangleMesh.create_sphere(radius=0.1)
        rate_gyro_sphere.translate(turn_axis)
        
        accelerometer_point_cloud = op.geometry.PointCloud()
        accelerometer_point_cloud.points = op.utility.Vector3dVector(self.accelerometer_data)
        accelerometer_point_cloud.paint_uniform_color([0.0, 1.0, 0.0])
        accel_sphere = op.geometry.TriangleMesh.create_sphere(radius=0.1)
        accel_sphere.translate(gravity)

        #5.0
        frame = op.geometry.TriangleMesh.create_coordinate_frame(size=2.0, origin=[0,0,0])

        #op.visualization.draw_geometries([magnetometer_point_cloud, frame, circle_point_cloud])
        if self.qc:
            self.qc_draw_geometries([frame,
                                     magnetometer_point_cloud, torus,
                                     rate_gyro_point_cloud, accelerometer_point_cloud,
                                     rate_gyro_sphere, accel_sphere])
        else:
            op.visualization.draw_geometries([frame,
                                              magnetometer_point_cloud, torus,
                                              rate_gyro_point_cloud, accelerometer_point_cloud,
                                              rate_gyro_sphere, accel_sphere])
        
    def visualize_data(self):
        # magnetometer_point_cloud = op.geometry.PointCloud()
        # magnetometer_point_cloud.points = op.utility.Vector3dVector(self.magnetometer_data)
        # #frame = op.create_mesh_coordinate_frame(size=100.0, origin=new_center.flatten())
        # #op.visualization.draw_geometries([magnetometer_point_cloud, frame, circle_point_cloud])
        # op.visualization.draw_geometries([magnetometer_point_cloud])

        rate_gyro_point_cloud = op.geometry.PointCloud()
        print('rate_gyro_data.shape =', self.rate_gyro_data.shape)
        rate_gyro_point_cloud.points = op.utility.Vector3dVector(self.rate_gyro_data)
        #frame = op.create_mesh_coordinate_frame(size=100.0, origin=new_center.flatten())
        #op.visualization.draw_geometries([rate_gyro_point_cloud, frame, circle_point_cloud])
        rate_gyro_point_cloud.paint_uniform_color([0.0, 0.0, 1.0])
        
        frame = op.geometry.TriangleMesh.create_coordinate_frame(size=5.0, origin=[0,0,0])

        base_velocity = self.mobile_base_data[:,1]
        n = len(base_velocity)
        print('n =', n)
        y_axis = np.array([0.0, 1.0, 0.0])
        y_axes = np.tile(y_axis, (n, 1))
        print('y_axes.shape =', y_axes.shape)
        base_velocity_vectors = y_axes * np.reshape(base_velocity, (n,1))
        print('base_velocity_vectors.shape =', base_velocity_vectors.shape)
        base_velocity_point_cloud = op.geometry.PointCloud()
        base_velocity_point_cloud.points = op.utility.Vector3dVector(base_velocity_vectors)
        base_velocity_point_cloud.paint_uniform_color([1.0, 0.0, 0.0])
        
        #op.visualization.draw_geometries([rate_gyro_point_cloud, base_velocity_point_cloud, frame])
        op.visualization.draw_geometries([rate_gyro_point_cloud, base_velocity_point_cloud])
        
    def qc_draw_geometries(self,geometry):
        # The following code achieves the same effect as:
        # o3d.visualization.draw_geometries([pcd])
        vis = op.visualization.Visualizer()
        vis.create_window()
        for g in geometry:
            vis.add_geometry(g)
        ctr = vis.get_view_control()

        #x (float): Distance the mouse cursor has moved in x-axis.
        #y (float): Distance the mouse cursor has moved in y-axis.
        #xo (float, optional, default=0.0): Original point coordinate of the mouse in x-axis.
        #yo (float, optional, default=0.0): Original point coordinate of the mouse in y-axis.

        # Do a single render at a set viewpoint, saves the image, and exit non blocking
        if self.validation_run:
            ctr.rotate(0.0, 250.0)
            # vis.run()
            vis.poll_events()
            vis.update_renderer()
            vis.capture_screen_image('base_imu_calibration_calibrated_fit.png')
        else:
            ctr.rotate(0.0, 1200.0)
            #vis.run()
            vis.poll_events()
            vis.update_renderer()
            vis.capture_screen_image('base_imu_calibration_uncalibrated_fit.png')
        vis.destroy_window()

    def calculate_error(self, parameters_to_fit):
        center, normal, radius, turn_axis, gravity = self.unpack_parameters(parameters_to_fit)
        #print('magnetometer_data =', self.magnetometer_data)
        #print('magnetometer_data.shape =', self.magnetometer_data.shape)
        from_center = self.magnetometer_data - center.flatten()
        #print('from_center =', from_center)
        #print('from_center.shape =', from_center.shape)
        # dot products
        height_error = np.dot(normal.transpose(), from_center.transpose())
        #print('height_error =', height_error)
        #print('height_error.shape =', height_error.shape)
        #temp = height_error * normal
        #print('temp.shape =', temp.shape)
        radial_error = np.linalg.norm(from_center.transpose() - (height_error * normal), axis=0)
        #print('radial_error =', radial_error)
        #print('radial_error.shape =', radial_error.shape)
        radial_error = np.abs(radial_error - radius)
        #print('radial_error =', radial_error)
        #print('radial_error.shape =', radial_error.shape)
        height_error = np.abs(height_error)
        #print('height_error =', height_error)
        #print('height_error.shape =', height_error.shape)
        total_error = np.mean(height_error) + np.mean(radial_error)

        difference = self.rate_gyro_data - turn_axis.flatten()
        rate_gyro_error = np.linalg.norm(difference.transpose(), axis=0)
        rate_gyro_error = np.mean(rate_gyro_error)
        total_error += rate_gyro_error
        
        difference = self.accelerometer_data - gravity.flatten()
        gravity_error = np.linalg.norm(difference.transpose(), axis=0)
        gravity_error = np.mean(gravity_error)
        total_error += gravity_error
        
        parameter_error = self.parameter_error_term(parameters_to_fit)
        total_error += parameter_error
        
        return total_error

    def post_fit_processing(self, final_parameters):
        center, normal, radius, turn_axis, gravity = self.unpack_parameters(final_parameters)
        percentile = 0.8
        
        # find 80th percentile radius for rate gyro turn vector
        difference = self.rate_gyro_data - turn_axis.flatten()
        rate_gyro_error = np.linalg.norm(difference.transpose(), axis=0)
        sorted_indices = np.argsort(rate_gyro_error)
        rate_gyro_index = int(round(percentile * len(sorted_indices)))
        rate_gyro_radius = sorted_indices[rate_gyro_index]

        # find 80th percentile radius for accelerometer gravity vector
        difference = self.accelerometer_data - gravity.flatten()
        gravity_error = np.linalg.norm(difference.transpose(), axis=0)
        sorted_indices = np.argsort(gravity_error)
        gravity_index = int(round(percentile * len(sorted_indices)))
        gravity_radius = sorted_indices[gravity_index]
        
        # find the start time for the turn
        for i, r in enumerate(self.rate_gyro_data):
            if np.linalg.norm(r - turn_axis.flatten()) <= rate_gyro_radius:
                start_time = self.time_data[i]
                start_angle = self.mobile_base_data[i,0]
                start_index = i
                break

        # find the end time for the turn
        for i, r in enumerate(self.rate_gyro_data[::-1]):
            if np.linalg.norm(r - turn_axis.flatten()) <= rate_gyro_radius:
                end_time = self.time_data[::-1][i]
                end_angle = self.mobile_base_data[::-1][i,0]
                end_index = (len(self.time_data) - 1) - i
                break

        # find the turn duration
        turn_duration = end_time - start_time

        print('turn lasted = {0} s'.format(turn_duration))
        print('start_angle = {0} deg'.format(180.0 * (start_angle/np.pi)))
        print('end_angle = {0} deg'.format(180.0 * (end_angle/np.pi)))

        turn_angles = self.mobile_base_data[start_index:(end_index+1),0]
        # find the turn direction
        sign_sum = 0
        for a0, a1 in zip(turn_angles, turn_angles[1:]):
            sign_sum += np.sign(a1 - a0)
        turn_sign = np.sign(sign_sum)
        # find the total turn angle using a dirty brute force method
        # to avoid rollover and range convention issues
        turn_angle = 0.0
        for a0, a1 in zip(turn_angles, turn_angles[1:]):
            turn_angle += angle_diff_rad(a1, a0)
        turn_angle = turn_sign * abs(turn_angle)
        print('turn angle = {0} deg'.format(180.0 * (turn_angle/np.pi)))
        rad_per_s = turn_angle / turn_duration
        print('turn velocity = {0} deg/s'.format(180.0 * (rad_per_s/np.pi)))

        # find the scale factor for the rate gyro
        rate_gyro_scale = rad_per_s / np.linalg.norm(turn_axis)
        print('rate gyro vector scale = {0}'.format(rate_gyro_scale))
                
        # find the scale factor for the gravitational direction of the accelerometer
        gravity_scale = 9.80665 / np.linalg.norm(gravity)
        print('gravity vector scale = {0}'.format(gravity_scale))

        if self.qc:
            results={'turn_duration':float(turn_duration),
                    'turn_angle':float(180.0 * (turn_angle/np.pi)),
                    'turn_velocity':float(180.0 * (rad_per_s/np.pi)),
                    'rate_gyro_scale':float(rate_gyro_scale),
                    'gravity_scale':float(gravity_scale)}
            if self.validation_run:
                with open('base_imu_calibrated_results.yaml', 'w') as yaml_file:
                    yaml.dump(results, yaml_file, default_flow_style=False)
            else:
                with open('base_imu_uncalibrated_results.yaml', 'w') as yaml_file:
                    yaml.dump(results, yaml_file, default_flow_style=False)


        return rate_gyro_scale, gravity_scale
        

    def print_data(self):
        print(self.data)


    
if __name__ == '__main__':

    try:

        parser = ap.ArgumentParser(description='Process mobile bas IMU calibration data and work with resulting files.')
        parser.add_argument('--load', action='store', help='Do not perform an optimization and instead load the specified file, which should contain CMA-ES optimization results.', default=None)
        parser.add_argument('--load_prev', action='store_true', help='Do not perform an optimization and instead load the most recent CMA-ES optimization results.')
        parser.add_argument('--only_vis', action='store_true', help='Only visualize the fit of the CMA-ES optimization results. This does not save any results.')
        parser.add_argument('--no_vis', action='store_true', help='Do not calculate or publish any visualizations. This results in faster fitting.')
        parser.add_argument('--validate', action='store_true', help='Fit and visualize most recent validation run.')
        parser.add_argument('--qc', action='store_true', help='QC script mode.')

        args, unknown = parser.parse_known_args()
        opt_results_file_to_load = args.load
        load_most_recent_opt_results = args.load_prev
        visualize_only = args.only_vis
        turn_on_visualization = not args.no_vis
        validation_run = args.validate

        calibration_directory = hu.get_fleet_directory()+'calibration_base_imu/'

        sample_selector_func = use_all_data
        #sample_selector_func = use_very_little_data
        visualize = turn_on_visualization
        
        if load_most_recent_opt_results or (opt_results_file_to_load is not None): 
            if load_most_recent_opt_results: 
                filenames = glob.glob(calibration_directory + hu.get_fleet_id()+'_base_imu_calibration_result' + '_*[0-9].yaml')
                filenames.sort()
                filename = filenames[-1]
                print('Loading most recent CMA-ES result from a YAML file named ' + filename)
            else:
                filename = opt_results_file_to_load
                print('Loading CMA-ES result from a YAML file named ' + filename)
            fid = open(filename, 'r')
            cma_result = yaml.load(fid)
            fid.close()

            calibration_options = cma_result.get('calibration_options', {})
            fit_parameters = np.array(cma_result['best_parameters'])
            calibrator = MobileBaseImuCalibrator(calibration_directory, sample_selector_func, calibration_options, visualize, validation_run,args.qc)
            
            if visualize_only:
                print('Loading the most recent data file.')
                calibrator.load_data()
                print('Visualizing how well the model fits the data.')
                calibrator.visualize_fit(fit_parameters)
            else: 
                print('Not Implemented: Should update files based on previous optimization that was loaded.')
        else:
            calibration_options = {}
            calibrator = MobileBaseImuCalibrator(calibration_directory, sample_selector_func, calibration_options, visualize, validation_run,args.qc)
            parameter_names = calibrator.get_names_of_parameters_to_fit()

            # different error tolerances for different speeds and qualities of fit
            fast_low_quality_tolfun = 0.1
            medium_tolfun = 0.01
            slow_high_quality_tolfun = 0.001
            #slowest_high_quality_tolfun = 0.0001
            slowest_high_quality_tolfun = 0.000001
            #cma_tolfun = fast_low_quality_tolfun
            #cma_tolfun = medium_tolfun
            cma_tolfun = slowest_high_quality_tolfun
            
            #incumbent solution:
            all_options = cma.CMAOptions()
            #"termination criterion: tolerance in function value"
            options = {'tolfun': cma_tolfun}

            calibrator.load_data()
            num_parameters = len(parameter_names)
            #initial_solution = num_parameters * [0.0]
            initial_solution = calibrator.initial_parameters()
            calibrator.set_soft_constraints(initial_solution)
            #calibrator.print_data()
            #calibrator.visualize_data()
            #exit()
            #initial_solution[parameter_names.index('pan_angle_scale')] = 1.0
            initial_standard_deviation = 0.1
            es = cma.CMAEvolutionStrategy(initial_solution, initial_standard_deviation, options)
            es.optimize(calibrator.calculate_error)
            print
            print('Optimization complete.')
            print
            es.result_pretty()
            
            # A results tuple from CMAEvolutionStrategy property result.

            # This tuple contains in the given position and as attribute
            
            #     0 xbest best solution evaluated
            #     1 fbest objective function value of best solution
            #     2 evals_best evaluation count when xbest was evaluated
            #     3 evaluations evaluations overall done
            #     4 iterations
            #     5 xfavorite distribution mean in "phenotype" space, to be considered as current best estimate of the optimum
            #     6 stds effective standard deviations, can be used to compute a lower bound on the expected coordinate-wise distance to the true optimum, which is (very) approximately stds[i] * dimension**0.5 / min(mueff, dimension) / 1.5 / 5 ~ std_i * dimension**0.5 / min(popsize / 2, dimension) / 5, where dimension = CMAEvolutionStrategy.N and mueff = CMAEvolutionStrategy.sp.weights.mueff ~ 0.3 * popsize.

            r = es.result
            best_parameters = r[0]
            rate_gyro_scale, gravity_scale = calibrator.post_fit_processing(best_parameters)
            if not args.no_vis:
            	calibrator.visualize_fit(best_parameters)
            
            no_numpy_cma_result = []
            for entry in es.result:
                if "tolist" in dir(entry):
                    entry = entry.tolist()
                no_numpy_cma_result.append(entry)

            no_numpy_initial_solution = initial_solution.tolist()
                
            cma_result = {'initial_solution': no_numpy_initial_solution,
                          'initial_standard_deviation': initial_standard_deviation,
                          'options': options,
                          'calibration_options': calibrator.get_calibration_options(), 
                          'parameter_names': parameter_names,
                          'best_parameters': no_numpy_cma_result[0],
                          'rate_gyro_scale': rate_gyro_scale.tolist(),
                          'gravity_scale': gravity_scale.tolist(),
                          'best_parameters_error': no_numpy_cma_result[1],
                          'num_evals_to_find_best': no_numpy_cma_result[2],
                          'num_evals_total': no_numpy_cma_result[3],
                          'cma_iterations': no_numpy_cma_result[4],
                          'cma_parameter_means': no_numpy_cma_result[5],
                          'cma_parameter_stddevs': no_numpy_cma_result[6]}

            time_string = create_time_string()

            filename = calibration_directory + hu.get_fleet_id()+'_base_imu_calibration_result_' + time_string + '.yaml'
            if not visualize_only and not validation_run:
                print()
                print('********************************************************')
                filename = calibration_directory + hu.get_fleet_id()+'_base_imu_calibration_result_' + time_string + '.yaml'
                print('Saving CMA-ES result to a YAML file named ', filename)
                fid = open(filename, 'w')
                yaml.dump(cma_result, fid)
                fid.close()
                print('Finished saving.')
                fit_parameters = cma_result['best_parameters']
                if args.qc:
                    write_configuration_params(cma_result)
                else:
                    x=input('Push parameters to stretch_configuration_params.yaml (y/n)? [y]')
                    if len(x)==0 or x=='y' or x=='Y':
                        print('Writing yaml...')
                        write_configuration_params(cma_result)
            else:
                print()
                print('********************************************************')
                print('Not saving due to visualize / validation only mode')
                print()
                print('Would have saved the CMA-ES result to a YAML file named ', filename)
                print()
                print('The following cma_result would have been saved:')
                print(cma_result)
        
    except (KeyboardInterrupt, SystemExit):
        pass



import numpy as np
from Adafruit_BNO055 import BNO055

class KalmanFilter:
    def __init__(self, initial_state_mean, initial_state_covariance, transition_matrix, observation_matrix,
                 process_noise_covariance, observation_noise_covariance):
        self.state_mean = initial_state_mean
        self.state_covariance = initial_state_covariance
        self.transition_matrix = transition_matrix
        self.observation_matrix = observation_matrix
        self.process_noise_covariance = process_noise_covariance
        self.observation_noise_covariance = observation_noise_covariance

    def predict(self):
        self.state_mean = np.dot(self.transition_matrix, self.state_mean)
        self.state_covariance = np.dot(np.dot(self.transition_matrix, self.state_covariance), self.transition_matrix.T) + self.process_noise_covariance

    def update(self, observation):
        observation_residual = observation - np.dot(self.observation_matrix, self.state_mean)
        observation_covariance = np.dot(np.dot(self.observation_matrix, self.state_covariance), self.observation_matrix.T) + self.observation_noise_covariance
        kalman_gain = np.dot(np.dot(self.state_covariance, self.observation_matrix.T), np.linalg.inv(observation_covariance))
        self.state_mean = self.state_mean + np.dot(kalman_gain, observation_residual)
        self.state_covariance = self.state_covariance - np.dot(np.dot(kalman_gain, self.observation_matrix), self.state_covariance)

# Create a BNO055 instance with a specified I2C address
bno = BNO055.BNO055(address=0x29, rst=18)  # Replace 0x29 with your actual device address

# Initialize the BNO055 sensor
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055!')

# Define the initial state of the system
initial_state_mean = np.zeros(12)  # Assuming 12-dimensional state
initial_state_covariance = np.eye(12)    # Assuming identity matrix as initial covariance

# Define the transition matrix (state transition model)
transition_matrix = np.eye(12)  # Assuming no change in state over time

# Define the observation matrix (observation model)
observation_matrix = np.eye(12)  # Assuming direct observation of state variables

# Define the process noise covariance (how much the system is expected to deviate from the model)
process_noise_covariance = np.eye(12) * 0.1  # Assuming small process noise

# Define the observation noise covariance (measurement noise)
observation_noise_covariance = np.eye(12) * 0.1  # Assuming small observation noise

# Create a Kalman filter instance
kf = KalmanFilter(initial_state_mean, initial_state_covariance, transition_matrix, observation_matrix,
                  process_noise_covariance, observation_noise_covariance)

# Apply Kalman filter to each measurement and output filtered values
while True:
    # Read sensor data
    heading, roll, pitch = bno.read_euler()
    gyro_x, gyro_y, gyro_z = bno.read_gyroscope()
    accel_x, accel_y, accel_z = bno.read_accelerometer()
    mag_x, mag_y, mag_z = bno.read_magnetometer()
    
    # Formulate the measurement vector
    measurement = np.array([gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, heading, roll, pitch])
    
    # Predict next state
    kf.predict()

    # Update with measurement
    kf.update(measurement)

    # Output filtered state
    print("Filtered state:")
    print("Gyroscope X (rad/s):", kf.state_mean[0])
    print("Gyroscope Y (rad/s):", kf.state_mean[1])
    print("Gyroscope Z (rad/s):", kf.state_mean[2])
    print("Accelerometer X (g):", kf.state_mean[3])
    print("Accelerometer Y (g):", kf.state_mean[4])
    print("Accelerometer Z (g):", kf.state_mean[5])
    print("Magnetometer X (uT):", kf.state_mean[6])
    print("Magnetometer Y (uT):", kf.state_mean[7])
    print("Magnetometer Z (uT):", kf.state_mean[8])
    print("Heading (degrees):", kf.state_mean[9])
    print("Roll (degrees):", kf.state_mean[10])
    print("Pitch (degrees):", kf.state_mean[11])
    print()

# Additional processing or control logic can be added as needed

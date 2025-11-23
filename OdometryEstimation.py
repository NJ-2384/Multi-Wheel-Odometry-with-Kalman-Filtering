import KalmanFilter

class OdometryEstimation(object):
    def __init__(self, process_variance, measurement_variance):
        self.kalman_filter_x = KalmanFilter.KalmanFilter(process_variance, measurement_variance)
        self.kalman_filter_y = KalmanFilter.KalmanFilter(process_variance, measurement_variance)
        self.kalman_filter_theta = KalmanFilter.KalmanFilter(process_variance, measurement_variance)
        self.kalman_filter_velocity_x = KalmanFilter.KalmanFilter(process_variance, measurement_variance)
        self.kalman_filter_velocity_y = KalmanFilter.KalmanFilter(process_variance, measurement_variance)
        self.kalman_filter_omega = KalmanFilter.KalmanFilter(process_variance, measurement_variance)
    
    def predict(self):
        self.kalman_filter_x.predict()
        self.kalman_filter_y.predict()
        self.kalman_filter_theta.predict()
        self.kalman_filter_velocity_x.predict()
        self.kalman_filter_velocity_y.predict()
        self.kalman_filter_omega.predict()
    
    def update(self, measurement):
        self.kalman_filter_x.update(measurement[0])
        self.kalman_filter_y.update(measurement[1])
        self.kalman_filter_theta.update(measurement[2])
        self.kalman_filter_velocity_x.update(measurement[3])
        self.kalman_filter_velocity_y.update(measurement[4])
        self.kalman_filter_omega.update(measurement[5])
    
    def get_estimate(self):
        estimate_x = self.kalman_filter_x.get_estimate()
        estimate_y = self.kalman_filter_y.get_estimate()
        estimate_theta = self.kalman_filter_theta.get_estimate()
        estimate_velocity_x = self.kalman_filter_velocity_x.get_estimate()
        estimate_velocity_y = self.kalman_filter_velocity_y.get_estimate()
        estimate_omega = self.kalman_filter_omega.get_estimate()
        
        return (estimate_x, estimate_y, estimate_theta, estimate_velocity_x, estimate_velocity_y, estimate_omega)


class KalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        
        self.posteriori_estimate = 0
        self.posteriori_error_estimate = 1
        self.priori_estimate = 0
        self.priori_error_estimate = 0
        self.kalman_gain = 0
        
    def predict(self):
        self.priori_estimate = self.posteriori_estimate
        self.priori_error_estimate = self.posteriori_error_estimate + self.process_variance
    
    def update(self, measurement):
        self.kalman_gain = self.priori_error_estimate / (self.priori_error_estimate + self.measurement_variance)
        self.posteriori_estimate = self.priori_estimate + self.kalman_gain * (measurement - self.priori_estimate)
        self.posteriori_error_estimate = (1 - self.kalman_gain) * self.priori_error_estimate
        
    def get_estimate(self):
        return self.posteriori_estimate
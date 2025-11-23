from multiprocessing import Process, Queue
import Kinematics
import OdometryEstimation
import time
import csv
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd
import sys

sys.setrecursionlimit(1000000000)


# Initial Conditions
INITIAL_ENCODER = (0,0,0,0)
INITIAL_ROBOT_POS = (0,0,0)
TIME_STEP = 1
PROCESS_VARIANCE = 1e-5
MEASUREMENT_VARIANCE = 1e-2
CURSOR_POSITION = 0

# DATA RATE (Hz)
OUTPUT_FREQUENCY = 50
FL_FREQUENCY = 120
FR_FREQUENCY = 100
BL_FREQUENCY = 150
BR_FREQUENCY = 120

# Queues
main_queue = Queue(maxsize=5000)
queue_fl = Queue(maxsize=5000)
queue_bl = Queue(maxsize=5000)
queue_fr = Queue(maxsize=5000)
queue_br = Queue(maxsize=5000)


def recieve_encoder_data(main_queue:Queue, queue_fl:Queue, queue_bl:Queue, queue_fr:Queue, queue_br:Queue):
    position_fl = CURSOR_POSITION
    position_fr = CURSOR_POSITION
    position_bl = CURSOR_POSITION
    position_br = CURSOR_POSITION

    def inside(position_fl, position_fr, position_bl, position_br, main_queue, queue_fl, queue_bl, queue_fr, queue_br):
        
        with open("Encoder_data/front_left.txt",'r') as fl:
            time.sleep(1/FL_FREQUENCY)
            fl.seek(position_fl)
            reading_fl = fl.readline().strip()
            position_fl = fl.tell()
            try:
                queue_fl.put(int(reading_fl))
            except:
                pass
            
        with open("Encoder_data/back_right.txt",'r') as br:
            time.sleep(1/BR_FREQUENCY)
            br.seek(position_br)
            reading_br = br.readline().strip()
            position_br = br.tell()
            try:
                queue_br.put(int(reading_br))
            except:
                pass
            
        with open("Encoder_data/back_left.txt",'r') as bl:
            time.sleep(1/BL_FREQUENCY)
            bl.seek(position_bl)
            reading_bl = bl.readline().strip()
            position_bl = bl.tell()
            try:
                queue_bl.put(int(reading_bl))
            except:
                pass
            
        with open("Encoder_data/front_right.txt",'r') as fr:
            time.sleep(1/FR_FREQUENCY)
            fr.seek(position_fr)
            reading_fr = fr.readline().strip()
            position_fr = fr.tell()
            try:
                queue_fr.put(int(reading_fr))
            except:
                pass

        
        while (queue_fl.empty() == False or queue_bl.empty() == False or queue_fr.empty() == False or queue_br.empty() == False):
            reading_fl = queue_fl.get()
            reading_bl = queue_bl.get()
            reading_fr = queue_fr.get()
            reading_br = queue_br.get()

            main_queue.put((reading_fl, reading_bl, reading_fr, reading_br))
        
        inside(position_fl, position_fr, position_bl, position_br, main_queue, queue_fl, queue_bl, queue_fr, queue_br)
    
    inside(position_fl, position_fr, position_bl, position_br, main_queue, queue_fl, queue_bl, queue_fr, queue_br)


def main(old_encoder_data, new_encoder_data, old_robot_pos, time_step, process_variance, measurement_variance):
    measured_values = Kinematics.ForwardKinematics(old_encoder_data, new_encoder_data, old_robot_pos, time_step)
    estimation = OdometryEstimation.OdometryEstimation(process_variance, measurement_variance)
    estimation.predict()
    estimation.update(measured_values)
    estimated_values = estimation.get_estimate()
    
    return estimated_values

def run(main_queue:Queue,output_frequency):
    time.sleep(2)
    
    old_encoder_value = INITIAL_ENCODER
    old_robot_pos = INITIAL_ROBOT_POS
    
    def inside(old_encoder_value, old_robot_pos,output_frequency):
        while (main_queue.empty() == False):
            
            # time.sleep(1/output_frequency)
            
            new_encoder_value = main_queue.get()
            estimated_value = main(old_encoder_value, new_encoder_value, old_robot_pos, TIME_STEP, PROCESS_VARIANCE, MEASUREMENT_VARIANCE)
            
            if (len(estimated_value) > 0):
                with open('Estimated_Values.csv', 'a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerows(((estimated_value),))
            
            old_encoder_value = tuple(new_encoder_value)
            old_robot_pos = (estimated_value[0], estimated_value[1], estimated_value[2])
            
        inside(old_encoder_value, old_robot_pos, output_frequency)
    
    inside(old_encoder_value, old_robot_pos, output_frequency)


def visualisation(i):
    data = pd.read_csv("Estimated_Values.csv")
    x = data['Estimated_x']
    y = data['Estimated_y']
    theta = data['Estimated_theta']

    plt.cla()
    
    plt.subplot(2,2,1)
    plt.plot(x, y, c='lightblue')
    plt.xlim(-500,500)
    plt.ylim(-500,500)
    plt.title("Robot Path")
    
    plt.subplot(2,2,2)
    plt.plot(x,c='red')
    plt.title("X_Estimate")
    
    plt.subplot(2,2,3)
    plt.plot(y,c='green')
    plt.title("Y_Estimate")
    
    plt.subplot(2,2,4)
    plt.plot(theta,c='black')
    plt.title("Theta_Estimate")
    
    plt.subplots_adjust(left=0.1,right=0.9,bottom=0.1,top=0.9,wspace=0.4,hspace=0.4)


def animate():
    time.sleep(1)
    ani = FuncAnimation(plt.gcf(), visualisation, interval=(1/OUTPUT_FREQUENCY))
    plt.show()
    
wheel = Process(target=recieve_encoder_data, args=(main_queue, queue_fl, queue_bl, queue_fr, queue_br))
final = Process(target=run, args=(main_queue, OUTPUT_FREQUENCY))
graph = Process(target=animate)

if __name__ == "__main__":
    wheel.start()
    final.start()
    graph.start()
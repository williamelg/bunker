#!/usr/bin/env python3

import psutil
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Initialisation 
measurement_interval = 0.1  #  seconds
time_span = 20  # time plot x-axis in seconds

# Initialize the plot
plt.rcParams.update({'font.size': 18})
fig, ax = plt.subplots(figsize=(8, 6))
line_cpu, = ax.plot([], [], linewidth=3, label='CPU')
line_gpu, = ax.plot([], [], linewidth=3, label='GPU')
ax.set_xlabel("Time (s)")
ax.set_ylabel("Usage (%)")
ax.set_xlim(0, time_span)
ax.set_ylim(0, 100)
ax.legend(loc='upper left')  

# Initialize the data
t = np.linspace(0, time_span, int(time_span / measurement_interval))
m_cpu = np.full_like(t, np.nan)
m_gpu = np.full_like(t, np.nan)

def update(frame):
    try:
        cpu_info = psutil.cpu_percent(interval=None)
        gpu_info = psutil.virtual_memory().percent  # Using psutil to get memory usage 

        # Update CPU data
        m_cpu[:-1] = m_cpu[1:]
        m_cpu[-1] = cpu_info
        line_cpu.set_data(t, m_cpu)

        # Update GPU data
        m_gpu[:-1] = m_gpu[1:]
        m_gpu[-1] = gpu_info
        line_gpu.set_data(t, m_gpu)

    except Exception as e:
        print(f"Error retrieving information: {e}")
    return line_cpu, line_gpu

ani = animation.FuncAnimation(fig, update, frames=None, blit=True, interval=measurement_interval * 1000)

plt.show()

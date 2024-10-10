import time
import random
import matplotlib.pyplot as plt

def measure_latency(num_trials=10):
    latencies = []
    for _ in range(num_trials):
        start_time = time.time()
        
        processing_delay = random.uniform(0.5, 1.5)  
        time.sleep(processing_delay)
        
        end_time = time.time()
        
        latency = end_time - start_time
        latencies.append(latency)
        
        print(f"Trial {_ + 1}: Latency = {latency:.3f} seconds")
    
    return latencies

def plot_latency(latencies):
    trials = range(1, len(latencies) + 1)
    plt.figure(figsize=(10, 6))
    plt.plot(trials, latencies, marker='o', linestyle='-', color='b')
    plt.xlabel('Trial Number')
    plt.ylabel('Latency (seconds)')
    plt.title('Voice Command Latency Over Multiple Trials')
    plt.grid(True)
    plt.show()

latency_data = measure_latency(num_trials=10)

plot_latency(latency_data)
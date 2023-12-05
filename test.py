import os

import time
import threading
import matplotlib.pyplot as plt

def function_to_run():
    while True:
        print("Hello World")

# plt.plot([1,2,3,4])
# plt.show(block = False)

start_time = time.time()

thread = threading.Thread(target=function_to_run)
thread.start()

if time.time() - start_time > 2.0:
    print("RTAAstar error,no result loop")
    # plt.close('all')
    os._exit(0)

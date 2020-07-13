from time import sleep, time
t_old = time()
while True:
    
    print("Test {0} \n".format(time() - t_old))
    t_old = time()
    sleep(0.001)

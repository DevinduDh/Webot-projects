#PID controller
from controller import Robot
from controller import LED

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#declare motor list
motor_list = ['front_right_wheel','front_left_wheel',
               'rear_right_wheel','rear_left_wheel']
#declare sensor list
sensor_list = ['ir_mid','ir_ext_right','ir_ext_left','ir_mid_right','ir_mid_left',
              'ir_ext_right_rear','ir_ext_left_rear','ir_mid_rear','ir_mid_right_rear','ir_mid_left_rear','ds_front','ds_right','ds_left']

led_list = ['led_0','led_1','led_2']
led = dict()
for l in led_list:
   led[l] = robot.getDevice(l)



motor = []

sensor = []
sensor_val = [0]*len(sensor_list)

for m in motor_list:
    motor.append(robot.getDevice(m))
    motor[-1].setPosition(float('inf'))
    motor[-1].setVelocity(0.0)

for s in sensor_list:
    sensor.append(robot.getDevice(s))
    sensor[-1].enable(timestep)



junction = 0
junction_val = 0

obst_right = 0
obst_right_val = 0

obst_left = 0
obst_left_val = 0

obst_both = 0
obst_both_val = 0

last_error = intg = diff = prop = waitCounter = 0
kp = 0.005
ki = 0
kd = 0.15

def pid(error):
    global last_error, intg, diff, prop, kp, ki, kd
    prop = error
    intg = error + intg
    diff = error - last_error
    balance = (kp*prop) + (ki*intg) + (kd*diff)
    last_error = error
    return balance

def setSpeed(base_speed, balance):
    motor[0].setVelocity(base_speed + balance)
    motor[1].setVelocity(base_speed - balance)
    motor[2].setVelocity(base_speed + balance)
    motor[3].setVelocity(base_speed - balance)

#def displayCount():


while robot.step(timestep) != -1:


    for i in range(len(sensor)):
        sensor_val[i] = sensor[i].getValue()
    led['led_2'].set(0)
    led['led_1'].set(0)
    led['led_0'].set(0)
    if(sensor_val[0]<300 and sensor_val[1]<300 and sensor_val[2]<300 and sensor_val[3]<300 and sensor_val[4]<300 and sensor_val[5]<300 and sensor_val[6]<300 and sensor_val[7]<300 and sensor_val[8]<300 ):
        setSpeed(0,0)
       
        #print(f'(sensor_list[i]) : (sensor_val[i])\n' + "*"*40)
    else:
        if(sensor_val[0]<300) or (sensor_val[3]<300) or (sensor_val[4]<300) or (sensor_val[7]<300) :
            #Sprint("in road")
            if(sensor_val[11]<950 and sensor_val[12] < 950):
                if obst_both_val == 0:
                    obst_both_val = 1
                    obst_both = obst_both + 1
                    print(obst_both)


                #print("both side obstacal")
            if(sensor_val[11] < 950):
                print("right obs")
                if obst_right_val == 0:
                    obst_right_val = 1
                    obst_right = obst_right + 1
                    print(obst_right)
                print("Right obstacle")
            if(sensor_val[12]< 950):
                if obst_left_val == 0:
                    obst_left_val = 1
                    obst_left = obst_left + 1
                    #print(obst_left)
                print("left obstacle")
        if(( (sensor_val[3]<300 and sensor_val[1]<300) and (sensor_val[4]>950 and sensor_val[2]>950))or( (sensor_val[3]>950 and sensor_val[1]>950) and (sensor_val[4]<300 and sensor_val[2]<300))):
            setSpeed(5,0)
            junction_val = 0
            obst_right_val = 0
            #print("inter face")

        elif(sensor_val[2]<300 and sensor_val[0]<300 and sensor_val[1]<300 and sensor_val[6]>950 and sensor_val[5]>950):
            if junction_val == 0:
                junction_val = 1
                junction = junction + 1
                #print(junction)

                led['led_2'].set(1)
                led['led_1'].set(1)
                led['led_0'].set(1)
            else:
                led['led_2'].set(1)
                led['led_1'].set(1)
                led['led_0'].set(1)

                setSpeed(5,0)
                #print("juction_forward")


        #elif(sensor_val[2]>950 and sensor_val[1]>950 and sensor_val[6]>950 and sensor_val[5]>950 and sensor_val[7]<300 and sensor_val[9]<300 and sensor_val[8]<300):
            #print("in_juction")

        elif(sensor_val[1]<300 and sensor_val[3]<300 and sensor_val[4]<300 and sensor_val[2]>950):
            #print("turn inter_ left")
            setSpeed(2,2)
            junction_val = 0
            obst_right_val = 0


        elif(sensor_val[1]>950 and sensor_val[3]<300 and sensor_val[4]<300 and sensor_val[2]<300):
            #print("turn inter_ right")
            setSpeed(2,-2)
            junction_val = 0
            obst_right_val = 0

        elif(sensor_val[1]<300 and sensor_val[3]>950 and sensor_val[4]>950 and sensor_val[2]>950):
            #print("turn inter_ right_ 1")
            setSpeed(2,-2)
            junction_val = 0
            obst_right_val = 0

        elif(sensor_val[1]<950 and sensor_val[3]>950 and sensor_val[4]>950 and sensor_val[2]<300):
            #print("turn inter_ left_1")
            setSpeed(2,2)
            junction_val = 0
            obst_right_val = 0
       # elif(sensor_val[2]>600 and sensor_val[2]<900 and sensor_val[])

        else:
            if sensor_val[0] < 300 and sensor_val[1] > 950 and sensor_val[2] > 950:
                #left and right sensor: blck ,center sensor: white
                #print("Go forward2")
                setSpeed(5,0)
                junction_val = 0
                obst_right_val = 0

            elif sensor_val[0] < 300 and sensor_val[1] < 300 and sensor_val[2] > 950:
                #right and mid sens: white left sens: black
                #print("Turn right1")
                setSpeed(2,-2)
                junction_val = 0
                obst_right_val = 0

            elif sensor_val[0] < 300 and sensor_val[1] > 950 and sensor_val[2] < 300:
                #left and mid sens: white ,right sens: blck
                #print("Turn left1")
                setSpeed(2,2)
                junction_val = 0
                obst_right_val = 0

            elif sensor_val[0] > 950 and sensor_val[1] < 300 and sensor_val[2] > 950:
                #left and mid sens: blck, right sens :white
                #print("Turn right2")
                setSpeed(2,-2)
                junction_val = 0
                obst_right_val = 0

            elif sensor_val[0] > 950 and sensor_val[1] > 950 and sensor_val[2] < 300:
                #right and mid sens: blck, left sens :white
                #print("Turn left2")
                setSpeed(2,2)
                junction_val = 0
                obst_right_val = 0

            else:


                if(sensor_val[12]!=1000):
                #wall in left

                    if(sensor_val[10]!=1000):
                    #obstacle in front
                       setSpeed(0,-6)
                       junction_val = 0
                       obst_right_val = 0

                    else:
                       error = sensor_val[12] - 595
                       balance = pid(error)
                       #print(balance)
                       setSpeed(5,balance)
                       junction_val = 0
                       #print("Hi2")
                       obst_right_val = 0
                elif(sensor_val[11]!=1000):
                #obstacle in right
                    if(sensor_val[10]!=1000):
                       #obstacle in front
                       setSpeed(0,6)
                       junction_val = 0
                       obst_right_val = 0
                       #turn left
                       #print("Hi3")
                    else:
                       error = sensor_val[11] - 595
                       balance = pid(error)
                       #print(balance)
                       setSpeed(5,balance*-1)
                       #print("Hi4")
                       junction_val = 0
                       obst_right_val = 0
                elif(sensor_val[10]!=1000):
                #obstacle in front
                       setSpeed(0,6)
                       junction_val = 0
                       obst_right_val = 0
                       #turn right
                       #print("Hi5")

                else:
                #no obstacle
                    setSpeed(5,0)
                    junction_val = 0
                    obst_right_val = 0
                    #print("Hi6")

    
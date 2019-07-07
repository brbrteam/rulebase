from drive_controller import DrivingController
import math

class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = True
        self.collision_flag = True
        
        #
        # Editing area ends
        # ==========================================================#
        super().__init__()
    
    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #



        # 중앙선과의 각도 ( - : 중앙선보다 왼쪽, + : 중앙선보다 오른쪽 )
        # 오른쪽으로 핸들꺾어야 하는 커if(sensing_info != null){
        # if(car_controls.steering !=0):
        #     car_controls.steering = -1 * car_controls.steering
        # else:
        #     car_controls.steering = 0
        # car_controls.throttle = 1
        # car_controls.brake = 0
        
        car_length = 2.5

        

        # to_middle + (car_length / 2) 가 절대로 self.half_road_limit - (car_length / 2) 를 넘어서는 안된다!

        flag = False


        # if(sensing_info.to_middle > 2):
        #     car_controls.steering = -0.05
        #     car_controls.brake = 0.05
        #     car_controls.throttle = 0.3
        #     flag = True
        # elif(sensing_info.to_middle <-2 ):
        #     car_controls.steering = 0.05
        #     car_controls.brake = 0.05
        #     car_controls.throttle = 0.3
        #     flag = True
        

        # if(flag==False and sensing_info.to_middle >= -1 and sensing_info.to_middle <=3):
        #     car_controls.throttle = 1
        #     car_controls.brake = 0
        #     car_controls.steering = 0
        #     flag = True
        
        if(car_controls.steering > 0.2):
            car_controls.steering = 0
            return car_controls

        if(sensing_info.speed >=0 and sensing_info.speed <=0.5):
            car_controls.steering = 0
            car_controls.brake = 0
            car_controls.throttle = 1
            return car_controls

        car_controls.brake = 0
        car_controls.throttle = 1
        car_controls.steering = 0

        middle = sensing_info.to_middle

        number = predictCurve(sensing_info)
        if(number>0):
            if(car_controls.steering != 0):
                car_controls.steering = 0
            else:
                if(sensing_info.to_middle < -3):
                    car_controls.steering = 0.03 * number - (0.01*middle)
                else: 
                    car_controls.steering = 0.01 * number - (0.01*middle)
            
        elif(number<0):
            if(car_controls.steering != 0):
                car_controls.steering = 0
            else:
                if(sensing_info.to_middle > 3):
                    car_controls.steering = 0.03 * number + (0.01*middle)
                else:
                    car_controls.steering = 0.01 * number + (0.01*middle)

        else:
            if(sensing_info.to_middle > 3):
                car_controls.steering = -(0.02*middle)
            elif(sensing_info.to_middle < -3):
                car_controls.steering = -(0.01*middle)
        
        if(sensing_info.speed > 30):
            car_controls.brake = 0
            car_controls.throttle = 0.5
        
        angle = sensing_info.moving_angle
        
        if(angle > 5 and car_controls.steering >0.03 ):
            # 이러면 잔디로 나가니까 다시 steering을 중간으로 맞춰야 한다.
            car_controls.steering = - car_controls.steering
        elif(angle < -5 and car_controls.steering < -0.03):
            car_controls.steering = - car_controls.steering
        
        car_controls.steering = find_obstacle(car_controls,sensing_info)


        if self.is_debug:
            print("\n도로의 넓이는 : {}".format(self.half_road_limit))
            print("=========================================================")
            print("to middle: {}".format(sensing_info.to_middle))

            print("collided: {}".format(sensing_info.collided))
            print("car speed: {} km/h".format(sensing_info.speed))

            print("is moving forward: {}".format(sensing_info.moving_forward))
            print("moving angle: {}".format(sensing_info.moving_angle))
            print("lap_progress: {}".format(sensing_info.lap_progress))

            print("track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            print("opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            print("=========================================================")
            print("다음 to_middle 예측 : {}".format(sensing_info.to_middle + (calculateFutureToMiddle(sensing_info))))
            print("=========================================================\n")
        ###########################################################################

        # Moving straight forward
        # car_controls.steering = 0
        # car_controls.throttle = 1
        # car_controls.brake = 0

        if self.is_debug:
            print("steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle, car_controls.brake))

        #
        # Editing area ends
        # ==========================================================#
        return car_controls


    # ============================
    # If you have NOT changed the <settings.json> file
    # ===> player_name = ""
    #
    # If you changed the <settings.json> file
    # ===> player_name = "My car name" (specified in the json file)  ex) Car1
    # ============================
    def set_player_name(self):
        player_name = ""
        return player_name

    
def calculateFutureToMiddle(sensing_info):
        angle = abs(sensing_info.moving_angle)
        secondPerMeter = (sensing_info.speed * 1000) / 3600
        future =  math.tan(math.pi * (angle/180)) * secondPerMeter

        return future

def predictCurve(sensing_info):
    arr = sensing_info.track_forward_angles
    
    next10_20 = arr[1] - arr[0]
    next20_30 = arr[2] - arr[1]

    return arr[0]

def find_obstacle(car_controls,sensing_info):
    nowSteering = car_controls.steering
    obs = sensing_info.track_forward_obstacles
    print(len(obs))

    if(len(obs) > 0):
        print(obs)
        firstObs = obs[0]

        if(firstObs["dist"] < 10):
            if(firstObs["to_middle"] >0 and firstObs["to_middle"] > sensing_info.to_middle and car_controls.steering > 0 and sensing_info.to_middle > 0):
                # 장애물이 현재 차보다 오른쪽에 있는데, 오른쪽으로 꺾으려고 하면!
                if(car_controls.steering > 0):
                    return -0.05
                else:
                    return -car_controls.steering
            elif(firstObs["to_middle"] < 0 and firstObs["to_middle"] < sensing_info.to_middle and car_controls.steering < 0 and sensing_info.to_middle <0 ):
                if(car_controls.steering < 0):
                    return 0.05
                else:

                    return -car_controls.steering

    return car_controls.steering    

if __name__ == '__main__':
    client = DrivingClient()
    client.run()

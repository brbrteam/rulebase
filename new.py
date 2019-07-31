from drive_controller import DrivingController
import math
import numpy

# class Car():
#     self.steering = 0
#     self.throttle = 1
#     self.brake = 0

class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #
        self.car_width = 2.5
        
        self.road = []
        self.steering_array = []
        self.flag = [True,True,True,True,True,True,True,True,True,True,True,True,True,True,True]
        self.prev_flag = False
        self.prev_steering = 0

        self.prev_angle = 0
        
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

        car_controls.steering = 0
        car_controls.throttle = 1
        
        road_width = self.half_road_limit - (self.car_width / 2)

        # init ---------------------------------------------------------------------------

        self.road = self.init_road(road_width)
        self.flag = [True,True,True,True,True,True,True,True,True,True,True,True,True,True,True]
        self.steering_array = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

        # --------------------------------------------------------------------------------
        
        # 현재 sensing_info

        middle = sensing_info.to_middle
        speed = sensing_info.speed
        angle = sensing_info.moving_angle
        forward_angles = sensing_info.track_forward_angles
        obstacles = sensing_info.track_forward_obstacles

        index = self.findIndex(middle)[0]

        if(self.isRight(angle) == False):

            print("isRight()")

            self.prev_angle = angle
            car_controls.steering = self.prev_steering
            car_controls.throttle = 1
            return car_controls

        # 차가 나갔나 안나갔나
        flag = self.isCarOut(middle,angle)
        
        # 현재 중앙으로 가기 위한 steering 을 계산

        self.findTrackCurve(angle,middle,forward_angles,speed)

        # 앞에 장애물이 있는지 확인 

        self.findObstacles(obstacles)

        # 만약 차가 지금 나갔다면??
        # 현재 위치에서 도로의 중앙까지 steering을 계산해서 주고 간다.

        if(flag):
            index = 7
            car_controls.steering = -self.steering_array[index]
            self.prev_steering = car_controls.steering
            self.prev_flag = self.flag
            return car_controls

        if(flag == True and self.prev_flag == False):
            car_controls.steering = - self.prev_steering
            self.prev_flag = self.flag
            return car_controls

        # 만약 1초 후에 갈 위치까지 obstacle이 없으면, 그냥 고고
        tmp_index = index
        
        if(self.flag[tmp_index] == False):
            if(index - 7 >=0):
                for i in range(tmp_index, 0 ,-1):
                    if(self.flag[i]):
                        index = i
                        break
            else:
                for i in range(tmp_index, 16 ,1):
                    if(self.flag[i]):
                        index = i
                        break


        

        # throttle 이 1이면 속도가 9km/h ~ 13km/h 증가한다. ( 9는 아마 잔디를 달릴때로 추정 )
        # brake 0.5가 10km/h를 줄이는 것으로 추정.

        # self.findObstacles(sensing_info.track_forward_obstacles)
        
        
        # car_controls.steering = self.steering_array[7]

        
        car_controls.steering = self.steering_array[index]
        self.prev_steering = car_controls.steering

        if(car_controls.steering > 0.15):
            car_controls.brake = 0.1



        # 이전 꺼 기억하자 ##############################################################
        
        self.prev_steering = car_controls.steering
        self.prev_flag = flag
        self.prev_angle = angle
        ################################################################################
        
        # Editing area ends
        # ==========================================================#
        print("####################################################")
        print("현재 road : {}".format(forward_angles))
        print("현재 car angle : {}".format(angle))
        print("현재의 steering : {}".format(car_controls.steering))
        print("####################################################\n")
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
        
    # 값이 비정상적으로 팅기나 안팅기나
    def isRight(self, angle):
        if(abs(self.prev_angle) - abs(angle) > 20 or abs(angle) - abs(self.prev_angle) > 20):
            return False

    # array
    # -10 ~ -7.5    -7.5 ~ -5        -5 ~ -2.5        -2.5 ~ 0        0 ~ 2.5       2.5 ~ 5        5 ~ 7.5        7.5 ~ 10
    #     -8.75 ~ -6.25   -6.25 ~ -3.75    -3.75 ~ -1.25   -1.25 ~ 1.25    1.25 ~ 3.75    3.75 ~ 6.25    6.25 ~ 8.75     

    def init_road(self,width):

        # -10 에서 10까지 1.25 간격으로~
        array = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        cnt = 0
        # 우선 도로가 아닌 곳을 막자!
        for i in numpy.arange(-10,-width,1.25):
            array[cnt] = -100
            cnt += 1
        
        cnt = 14
        for i in numpy.arange(10, width, -1.25):
            array[cnt] = -100
            cnt -= 1
        
        return array



    # 장애물의 위치에 따라 가중치를 부여한다.
    def findObstacles(self, array):
        length = len(array)
        print("findObstacles()")
        for i in range(0,length,1):
            obs = array[i]
            dist = obs["dist"]
            middle = obs["to_middle"]

            arr = self.findIndex(middle)
            for j in range(0,len(arr),1):
                idx = arr[j]
                print(idx)
                self.flag[idx] = False
        

            

    # number 위치일 때, road 배열의 어느 index인지 판단!
    def findIndex(self, number):
        arr = []
        result = number / 1.25

        if(result >= 0):
            index = 7+math.floor(abs(result))
            arr.append(index)
            arr.append(index+1)
        else:
            index = 7 - math.floor(abs(result))
            arr.append(index-1)
            arr.append(index)
        
        return arr






    # 도로의 방향에 따라 가중치를 부여한다.
    def findTrackCurve(self, nowAngle, nowMiddle, array, speed):
        length = len(array)
        
        # 현재 위치
        nowIndex = self.findIndex(nowMiddle)

        angle = array[math.floor(self.predictDistance(speed))]
        # throttle 부터 계산해보자.

        speedValue = 0
        speedAngle = 1
        if(speed > 100):
            speedValue = 0.025
            speedAngle = 0
        elif(speed > 50):
            speedValue = 0.02
            speedAngle = 1.3
        else:
            speedValue = 0.015
            speedAngle = 1.1

        # 우선 car angle 신경쓰지말고 짜보자.
        # 현재 road angle 이 양수야.
        
        
        nowAngle = nowAngle / 90 * speedAngle
        rad = math.radians(angle)
        tanz = rad
        self.steering_array[7] =  speedValue * (rad) - nowAngle
        
        
        for i in range(0,7,1):
            self.steering_array[i] = speedValue * math.atan(tanz + 0.125) * math.pi -nowAngle
            tanz = tanz + 0.125
        
        for i in range(8,length+1,1):
            self.steering_array[i] = speedValue * math.atan(tanz - 0.125) * math.pi -nowAngle
            tanz = tanz - 0.125
            if(tanz < 0):
                break
        



        

    # 제일 작은 index를 찾자
    def find(self):
        length = len(self.road)
        idx = 0
        maxValue = -999
        for i in range(0,length,1):
            if(maxValue < self.road[i]):
                idx = i
                maxValue = self.road[i]
        
       
        go = -10 + (idx+1) * 1.25

        print("우리가 가야할 곳은 {} 이다.".format(go))

        return go


     
    # 도로 밖으로 나가는지 판단!

    def isCarOut(self, middle, angle):
        print("isCarOut()")
        car = 1.25

        if(abs(middle) > abs(self.half_road_limit)):
            return True

        return False

        # to_middle + (car_length / 2) 가 절대로 self.half_road_limit - (car_length / 2) 를 넘어서는 안된다!

        # if(middle > 0 and (middle + car) > road):
        #     # 오른쪽이 도로를 나가려고 하면! 
        #     if(steering > 0):
        #         # 오른쪽으로 커브를 돌려고 하면! 왼쪽으로 가야하니까 왼쪽으로 간다!
        #         return 0
        # elif(middle <0 and (middle - car) < -road):
        #     # 왼쪽이 도로를 나가려고 하면!
        #     if(steering < 0):
        #         return 0

        # return steering

    
    # 속도와 자동차 방향을 계산해서 to_middle을 예측
    # steering = 0 이라고 가정 한 상태이다.
    def calculateFutureToMiddle(self,sensing_info, throttle, brake):

        speed = sensing_info.speed
        now = sensing_info.to_middle
        angle = sensing_info.moving_angle

        futureSpeed = speed + (throttle * 12) - (0.8 * brake)

        secondPerMeter = (futureSpeed * 1000) / 3600

        future =  math.tan(math.pi * (abs(angle)/180)) * secondPerMeter
        if(now*angle >0):
            if(angle > 0 ):
                return future + now
            else:
                return now - future
        else:
            if(angle>0):
                future =  math.tan((math.pi/2) - math.pi * (abs(angle)/180)) * secondPerMeter
                return now + future 
            else:
                return now - future
            
            

    # 현재 속도로 0.1초 후에 갈 커브 곡선 예측
    def predictDistance(self, speed):

        secondPerMeter = (speed * 1000) / 3600

        return secondPerMeter * 0.1
   

if __name__ == '__main__':
    client = DrivingClient()
    client.run()

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

        road_width = self.half_road_limit - (self.car_width / 2)

        road = self.init_road(road_width)

        print(road)

        # throttle 이 1이면 속도가 9km/h ~ 13km/h 증가한다. ( 9는 아마 잔디를 달릴때로 추정 )
        # brake 0.5가 10km/h를 줄이는 것으로 추정.

        
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
        weight = -5
        for i in range(0,length,1):
            obs = array[i]
            

    # number 위치일 때, road 배열의 어느 index인지 판단!
    def findIndex(self, number):
        arr = []
        result = number / 1.25

        if(result >= 0):
            index = 7+result
            arr.append(index-1)
            arr.append(index)
        else:
            index = 7 - math.floor(abs(result))
            arr.append(index-1)
            arr.append(index)
        
        return array











     
    # 도로 밖으로 나가는지 판단!

    def isCarOut(self, predict, angle):


        
        
        car = 1.25

        # 만약 예측 값이 self.half_road_limit을 넘어가려 한다??

        if(predict> self.half_road_limit):
            if(angle > 0):
                return [-0.05 , 0.6, 0.1]
        elif(predict < -self.half_road_limit):
            if(angle < 0):
                return [0.05 , 0.6, 0.1]

        return [0,1,0]   
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
            
            


   

if __name__ == '__main__':
    client = DrivingClient()
    client.run()

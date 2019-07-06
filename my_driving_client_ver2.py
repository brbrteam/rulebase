import math

from drive_controller import DrivingController


class DrivingClient(DrivingController):

    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.debug_controls_only = 1
        self.debug_infos = 2
        self.debug_all_trace = 3

        # 1 : all_trace / 2 : infos / 3 : steer, brake, throttle / 0 : no debugging
        self.debug_level = 0
        
        self.collision_flag = True
        self.half_car_size = 1.25
        
        self.max_val = 99999
        
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

        if self.debug_level >= self.debug_infos:
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
            print("half_road_size : {}".format(self.half_road_limit))
            print("=========================================================")

        ###########################################################################



        curve_infos = []
        curve_infos = self.calc_target_road(sensing_info)

        if self.debug_level >= self.debug_all_trace:
            print(curve_infos)

        self.make_decision(sensing_info, car_controls, curve_infos)

        if self.debug_level >= self.debug_controls_only:
            print("steering:{}, throttle:{}, brake:{}".format(round(car_controls.steering,2), round(car_controls.throttle,2), round(car_controls.brake,2)))   

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

    
    def make_decision(self, sensing_info, car_controls, curve_infos) :
        #road racing

        #steer
        min_info = [0, self.max_val, 0, self.max_val] #index, dist, angle, priority
        loop_index = 0
        for target_point in curve_infos[0]:
            if target_point[2] < min_info[3]: #priority penalty
                min_info[1] = target_point[0]
                min_info[2] = target_point[1]
                min_info[3] = target_point[2]
                min_info[0] = loop_index
            
            loop_index +=1

        if self.debug_level >= self.debug_infos:
            print("target = {}, dist = {}, angle = {}, priority = {}".format(min_info[0], min_info[1], min_info[2], min_info[3]))

        
        steer_value = min_info[2] * 0.015 # Rule of Thumb 0.015
        if steer_value < -1 :
            car_controls.steering = -1
        elif steer_value > 1 :
            car_controls.steering = 1
        else:
            car_controls.steering = steer_value
        
        if self.debug_level >= self.debug_infos:
            print("angle = {}, steering = {}".format(min_info[2], car_controls.steering))


        # brake
        brake_value = (math.pow(1.2, abs(car_controls.steering) * sensing_info.speed) -3)/5 # Rule of Thumb 1.2, 3, 5

        if brake_value < 0:
            car_controls.brake = 0
        elif brake_value > 1:
            car_controls.brake = 1
        else:
            car_controls.brake = brake_value
        
        if self.debug_level >= self.debug_infos:
            print("brake = {} , brake_value = {}".format(car_controls.brake, brake_value))

        # throttle
        
        if sensing_info.speed < 100 - min_info[2] :
            car_controls.throttle = 1   
            if self.debug_level >= self.debug_infos :
                print("throttle1. throttle = {}, brake = {}".format(car_controls.throttle, car_controls.brake))
        elif sensing_info.speed < 120 - min_info[2] :
            car_controls.throttle = 0.7
            if self.debug_level >= self.debug_infos :
                print("throttle2. throttle = {}, brake = {}".format(car_controls.throttle, car_controls.brake))
        else:
            car_controls.throttle = 0.5
            if self.debug_level >= self.debug_infos :
                print("throttle3. throttle = {}, brake = {}".format(car_controls.throttle, car_controls.brake))

        #     # out of road OR opponent car

    
    def calc_target_road(self,sensing_info) :
        # divide road data into 5 target_point
        # divided road are assumed to be parallel to road
        # track_forward_angles inforamtion is used for this calc
        
        #position caculation
        #매 계산시마다 현 구간의 middle을 원점 O로 정한다. 이후 다음 구간의 좌표를 구한다. 즉, 좌표는 한 구간 내에서만 상대적으로 유의미하다.
        #theta가 주어졌을때 현 위치의 middle과 다음 위치의 middle을 연결하는 선이 중심각 theta인 호로 가정한다. 
        #현재는 테스트로 5구간만 계산한다.

        curve_matrix_datas = [] #[][][] 구간 / target_point / 좌표정보
        curve_matrix_info = [] #판단용 전방 1구간
        curve_matrix_info_total = [] #dfs용 전체 구간


        before_angle = 0 #angle값 보정을 위함. E.G. 구간정보가 3, 10일경우, 첫 구간의 angle은 3, 이후 구간은 angle은 상대적으로 7.
        
        for angle in sensing_info.track_forward_angles:

            real_angle = angle - before_angle
            before_angle = angle

            l = 10
            theta = abs(math.radians(real_angle)) # angle 값이 음수일경우 계산이 복잡하므로 전부 양수처리 후 좌표 y 대칭이동
            r = 0
            
            if theta == 0:
                r = 0
            else :
                r = round(l/theta,2)

            # 부채꼴로 가정시, middle_a -> middle_b로 이어지는 선분은 이등변 삼각형의 밑변
            bottom_angle = (180-abs(real_angle))/2
            bottom_line = self.cosine_law(r, r, theta)

            # 도로 width를 고려하여, 차량이 있을 수 있는 midpoint 가능지점 산출
            # E.G.  full_road_width = 10[-5~5]. half_car_size = 1.25, target_point = [-3.75, -1.87, 0, 1.87 ,3.75]

            full_road_width = (self.half_road_limit - self.half_car_size)*2  
            interval = (full_road_width - 2*self.half_car_size)/5  #-2*interval ~ 0 ~ 2*interval. 총 5개 point이므로 4개구간. 이지만, 가동가능 범위(penalty) 넉넉히 피하기 위해 interval값을 줄이자.

            mid_x = 0
            mid_y = 0
            # theta에 따라 좌표 adjust시 사용할 변수.
            modif_x = 0
            modif_y = 0

            if real_angle > 0:
                mid_x = 1 * math.cos(math.radians(bottom_angle)) * bottom_line
                mid_y = math.sin(math.radians(bottom_angle)) * bottom_line
                modif_x = math.cos(theta) * interval
                modif_y = -1 * math.sin(theta) * interval
            elif real_angle <0:
                mid_x = -1 * math.cos(math.radians(bottom_angle)) * bottom_line
                mid_y = math.sin(math.radians(bottom_angle)) * bottom_line
                modif_x = math.cos(theta) * interval
                modif_y = 1 * math.sin(theta) * interval
            else:
                mid_x = 0
                mid_y = 10
                modif_x = interval
                modif_y = 0

            # 도로 5구간으로 구분.
            road_point_0 = [mid_x - 2*modif_x, mid_y - 2*modif_y]
            road_point_1 = [mid_x - 1*modif_x, mid_y - 1*modif_y]
            road_point_2 = [mid_x, mid_y]
            road_point_3 = [mid_x + 1*modif_x, mid_y + 1*modif_y]
            road_point_4 = [mid_x + 2*modif_x, mid_y + 2*modif_y]

            # curve_matrix_datas list에 각 포인트 좌표 저장
            curve_matrix_datas.append([road_point_0, road_point_1, road_point_2, road_point_3, road_point_4])
            
        
        # 장애물 발견시, curve_matrix_datas에 저장한 구간별 target point에 표시. target point로 선정되는걸 막는다.
        if len(sensing_info.track_forward_obstacles) != 0:
            for obs_info in sensing_info.track_forward_obstacles:

                obs_dist = obs_info.get('dist')
                obs_to_middle = obs_info.get('to_middle')
                
                # obs_dist가 35일경우, 0~10 구간, 11~20구간, 21~30구간, 31~40구간에 속하므로, 4구간, 즉 3번 index로 접근한다.
                # speed가 높을경우, 바로 앞 구간만 판단시 회피시간이 부족하므로, 보다 앞 데이터 또한 막아준다.
                # speed * 0.27 => m/s. 수행시 40 이상일 경우 1 구간으로 판단시 위험. 40km/h = 10.8m/s. 즉 1구간 진행시 1초 소요.
                # speed*1/40만큼 index를 당긴다.

                curve_matrix_index = max(int(obs_dist / 10), 0)
                curve_matrix_index_adj = max(int(obs_dist / 10 - sensing_info.speed * 0.04 ), 0)

                self.mark_obstacles(obs_dist, obs_to_middle, interval, curve_matrix_datas, curve_matrix_index_adj, curve_matrix_index)
                
                

        if self.debug_level >= self.debug_all_trace:
            print("curve_matrix_Datas")
            for matrix_data in curve_matrix_datas:
                print(matrix_data)

        # dfs 예정. 5^10 = 970만
        # 현재는 matrix data를 통해 distance와 angle을 얻고 curve_matrix_info에 저장.
        # 향후 우선순위 넣을것.

        ############## 테스트 코드 ###############
        
        distance_index = 0 # 구간 index
        for area in curve_matrix_datas :
            if distance_index == 0 : # 현 vehicle 기준 전방 1구간
                start_pos = sensing_info.to_middle
                start_angle = sensing_info.moving_angle
                result = []

                for target_point in area:
                    calced_dist = self.calc_dist(start_pos, 0, target_point[0], target_point[1])
                    calced_angle = self.calc_angle(start_pos, 0, target_point[0], target_point[1]) - sensing_info.moving_angle
                    calced_priority = self.calc_priority(calced_dist, calced_angle)

                    result.append([calced_dist, calced_angle, calced_priority])

                curve_matrix_info.append(result)

            else :
                pass

            distance_index += 1
            if distance_index >= 1:
                break

        return curve_matrix_info
                #curve_matrix_info_total에 집어넣을것.
                #curve_matrix_info_total = [][][][] : 구간번호 / 출발 target / 도착 target /  [dist, angle, calced_value, priority]



        #########################################


        ############## 운영코드 ##################
        # distance_index = 0
        # for matrix_data in curve_matrix_datas:

        #     if distance_index == 0: # 현 vehicle 기준
        #         start_pos = sensing_info.to_middle
        #         start_angle = sensing_info.moving_angle
        #         result = []

        #         for position in matrix_data:
        #             result.append([self.calc_dist(start_pos, 0, position[0], position[1]), self.calc_angle(start_pos, 0, position[0], position[1]) - start_angle ])

        #         curve_matrix_info.append(result)    
        #     else: # 각 target point 기준 전체탐색
        #         pass

        #     distance_index += 1
        #     if distance_index >=1:
        #         break
                
        # return curve_matrix_info

        #########################################
        

        

    def cosine_law(self, a, b, theta):
        # angle calculation impossible
        # estimate angle = 90 + theta/2
        result = math.sqrt(pow(a,2)+pow(b,2)-2*a*b*math.cos(theta))
        return result
    
    def calc_dist(self, start_x, start_y, target_x, target_y):
        if target_x == self.max_val and target_y == self.max_val:
            return self.max_val #dist max로 return 하여 dfs에서 선택되지 않도록.
        else:
            return round(math.sqrt(math.pow(target_x - start_x, 2) + math.pow(target_y - start_y, 2)),2)

    def calc_angle(self, start_x, start_y, target_x, target_y):
        result = 0
        if (target_x - start_x)*(target_y - start_y) > 0:
            # positive angle 
            result = 1 * (90 - math.degrees(math.atan(abs((target_y-start_y)/(target_x-start_x)))))

        elif (target_x - start_x)*(target_y - start_y) < 0:
            #negative angle

            result = -1 * (90 - math.degrees(math.atan(abs((target_y-start_y)/(target_x-start_x)))))

        else :
            result = 0

        return result

    def calc_priority(self, calced_dist, calced_angle):
        # distance와 angle이 주어졌을때, angle의 변화가 적고 distance가 적은 선택이 유리
        # 값을 어떻게 계산할 것인가... 
        # steering에 대한 속도감소값은 concave함. steering값이 커질수록 penalty의 증가량이 커져야함. calced_angle과 steering은 선형이므로 calced_angle 또한 concave로 가정.
        # distance에 대한 시간증가값은 linear함.
        # steering이 30 -> 40 , 40 -> 50, 50 -> 60이라면 penalty는 얼마나?
        # distance의 범위는 기껏해야 min~max 1정도.

        return calced_dist * math.pow(1.02, abs(calced_angle)/200)


    def mark_obstacles(self, obs_dist, obs_to_middle, interval, curve_matrix_datas, curve_matrix_index_adj, curve_matrix_index):
        for index in range(curve_matrix_index_adj, curve_matrix_index+1):
            if obs_to_middle + 1 <= -2*interval : # 장애물이 좌로 치우침
                curve_matrix_datas[index][0] = [self.max_val, self.max_val]
            elif obs_to_middle + 1 <= -1 * interval : #장애물이 좌에 있음
                curve_matrix_datas[index][0] = [self.max_val, self.max_val]
                curve_matrix_datas[index][1] = [self.max_val, self.max_val]
            elif obs_to_middle - 1 >= 2 * interval : #장애물이 우로 치우침
                curve_matrix_datas[index][4] = [self.max_val, self.max_val]
            elif obs_to_middle - 1 >= 1 * interval : #장애물이 우에 있음
                curve_matrix_datas[index][3] = [self.max_val, self.max_val]
                curve_matrix_datas[index][4] = [self.max_val, self.max_val]
            else : # 장애물이 중앙에 있음
                curve_matrix_datas[index][1] = [self.max_val, self.max_val]
                curve_matrix_datas[index][2] = [self.max_val, self.max_val]
                curve_matrix_datas[index][3] = [self.max_val, self.max_val]


if __name__ == '__main__':
    client = DrivingClient()
    client.run()

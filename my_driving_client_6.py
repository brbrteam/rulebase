import math
import copy
import time # 로깅을 위해 import함. 실 주행시 필요없음.

from drive_controller import DrivingController


class DrivingClient(DrivingController):

    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.DEBUG_CONTROLS_ONLY = 1
        self.DEBUG_INFOS = 2
        self.DEBUG_TRACE = 3
        self.DEBUG_ALL_RAWDATA = 4

        # 1 : controls_only / 2 : infos / 3 : trace / 4: all_raw_data / 0 : no debugging
        self.DEBUG_LEVEL = 0

        self.MAX_VAL = 99999
        self.MAX_VAL_100 = 9999999
        
        self.collision_flag = True
        self.HALF_CAR_SIZE = 1.25
        self.full_road_width = 0.0
        
        self.adjusted_track_forward_angle = []
        self.direct_waypoint_dist = 0
        self.speed_mps = 0
        self.decision_hist = [] #총 10회의 선택 기억. index, throttle, steer, brake
        self.HIST_LEVEL = 10
        self.race_start_time = time.time()

        
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
        # 도로 width를 고려하여, 차량이 있을 수 있는 target_point 가능지점 산출
        # E.G.  full_road_width = 10[-5~5]. HALF_CAR_SIZE = 1.25, target_point = [-3.75, -2.5, -1.25, 0, 1.25, 2.5, 3.75]


        self.full_road_width = (self.half_road_limit - self.HALF_CAR_SIZE)*2  
        self.direct_waypoint_dist = math.sqrt(math.pow(sensing_info.distance_to_way_points[0],2)-math.pow(sensing_info.to_middle, 2))       
        if len(self.decision_hist) == 0:
            self.decision_hist.append([self.MAX_VAL, self.MAX_VAL, self.MAX_VAL, self.MAX_VAL])

        self.adjusted_track_forward_angle = []
        self.speed_mps = round(sensing_info.speed * 1000 / 3600,2)
        
        for angle in sensing_info.track_forward_angles:
            self.adjusted_track_forward_angle.append(angle - sensing_info.moving_angle)        

        #self.interval = (self.full_road_width - 2*self.HALF_CAR_SIZE)/self.target_point_level

        if self.DEBUG_LEVEL >= self.DEBUG_INFOS:
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
            print("at race_time : {} sec".format(round(time.time()-self.race_start_time,2)))
            print("=========================================================")

        ###########################################################################



        
        self.make_decision(sensing_info, car_controls)

        if self.DEBUG_LEVEL >= self.DEBUG_CONTROLS_ONLY:
            print("final_control!! steering:{}, throttle:{}, brake:{}".format(round(car_controls.steering,2), round(car_controls.throttle,2), round(car_controls.brake,2)))   
        
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

    
    def make_decision(self, sensing_info, car_controls) :
        #road racing
       
        weight = 2
        min_total_angle = 0
        weighted_total_angle = 0
        for angle in sensing_info.track_forward_angles:
            min_total_angle += abs(angle)
            weighted_total_angle += abs(angle) * weight
            weight -= 0.1


        #steer        
        target_to_middle = self.calc_target_to_middle(sensing_info)
        angle_val = self.calc_target_angle(sensing_info, target_to_middle)

        #print("angle_val = {}".format(angle_val))

        steer_value = angle_val * 0.015  # 0번구간 target_point 까지의 angle * steer값

        if steer_value < -1 :
            car_controls.steering = -1
        elif steer_value > 1 :
            car_controls.steering = 1
        else:
            car_controls.steering = steer_value

        if self.DEBUG_LEVEL >= self.DEBUG_CONTROLS_ONLY:
            print("angle = {}, steering = {}".format(round(angle_val,2), round(car_controls.steering,2)))


        # throttle
        # 현재 속도와 angle값에 따라 throttle 조정        
        throttle_value = 0
        if sensing_info.speed < 160 - angle_val :
            throttle_value = 1   
            if self.DEBUG_LEVEL >= self.DEBUG_CONTROLS_ONLY:
                print("throttle1. throttle_value = {}, brake = {}".format(round(throttle_value,2), round(car_controls.brake,2)))
        elif sensing_info.speed < 140 - angle_val :
            throttle_value = 0.97
            if self.DEBUG_LEVEL >= self.DEBUG_CONTROLS_ONLY:
                print("throttle2. throttle_value = {}, brake = {}".format(round(throttle_value,2), round(car_controls.brake,2)))
        else:
            throttle_value = 0.95
            if self.DEBUG_LEVEL >= self.DEBUG_CONTROLS_ONLY:
                print("throttle3. throttle_value = {}, brake = {}".format(round(throttle_value,2), round(car_controls.brake,2)))

        # 전방 도로 커브에 따라 throttle 조정
        if sensing_info.speed <= 0 :
            throttle_value *= 1
        else: ## min_total_angle 100~200대면 완만한 커브 300 이상 큰 커브 600 이상 급커브
            if weighted_total_angle * math.pow(sensing_info.speed,2) < 2000000:
                throttle_value*=1
                if self.DEBUG_LEVEL >= self.DEBUG_CONTROLS_ONLY:
                    print("throttle_adjust_1. weighted_total_angle = {}, speed^2 = {}, value = {}".format(round(weighted_total_angle,2), round(math.pow(sensing_info.speed,2),2), round(weighted_total_angle * math.pow(sensing_info.speed,2), 2)))
            elif weighted_total_angle * math.pow(sensing_info.speed,2) < 2500000:
                throttle_value*=0.80
                if self.DEBUG_LEVEL >= self.DEBUG_CONTROLS_ONLY:
                    print("throttle_adjust_2. weighted_total_angle = {}, speed^2 = {}, value = {}".format(round(weighted_total_angle,2), round(math.pow(sensing_info.speed,2),2), round(weighted_total_angle * math.pow(sensing_info.speed,2), 2)))
            elif weighted_total_angle * math.pow(sensing_info.speed,2) < 3000000:
                throttle_value*=0.60
                if self.DEBUG_LEVEL >= self.DEBUG_CONTROLS_ONLY:
                    print("throttle_adjust_3. weighted_total_angle = {}, speed^2 = {}, value = {}".format(round(weighted_total_angle,2), round(math.pow(sensing_info.speed,2),2), round(weighted_total_angle * math.pow(sensing_info.speed,2), 2)))
            elif weighted_total_angle * math.pow(sensing_info.speed,2) < 3500000:
                throttle_value*=0.40
                if self.DEBUG_LEVEL >= self.DEBUG_CONTROLS_ONLY:
                    print("throttle_adjust_4. weighted_total_angle = {}, speed^2 = {}, value = {}".format(round(weighted_total_angle,2), round(math.pow(sensing_info.speed,2),2), round(weighted_total_angle * math.pow(sensing_info.speed,2), 2)))
            elif weighted_total_angle * math.pow(sensing_info.speed,2) < 4000000:
                throttle_value*=0.20
                if self.DEBUG_LEVEL >= self.DEBUG_CONTROLS_ONLY:
                    print("throttle_adjust_4. weighted_total_angle = {}, speed^2 = {}, value = {}".format(round(weighted_total_angle,2), round(math.pow(sensing_info.speed,2),2), round(weighted_total_angle * math.pow(sensing_info.speed,2), 2)))
            else :
                throttle_value*=0
                if self.DEBUG_LEVEL >= self.DEBUG_CONTROLS_ONLY:
                    print("throttle_adjust_5. weighted_total_angle = {}, speed^2 = {}, value = {}".format(round(weighted_total_angle,2), round(math.pow(sensing_info.speed,2),2), round(weighted_total_angle * math.pow(sensing_info.speed,2), 2)))


        if throttle_value <0 :
            car_controls.throttle = 0
        elif throttle_value >1:
            car_controls.throttle = 1
        else:
            car_controls.throttle = throttle_value

        # brake
        # steer와 speed에 따라 brake 조정
        brake_value = (math.pow(1.2, abs(car_controls.steering) * sensing_info.speed) -6)/200 

        if brake_value < 0:
            car_controls.brake = 0
        elif brake_value > 1:
            car_controls.brake = 1
        else:
            car_controls.brake = brake_value
        
        if self.DEBUG_LEVEL >= self.DEBUG_CONTROLS_ONLY:
            print("brake = {} , brake_value = {}".format(round(car_controls.brake,2), round(brake_value,2)))



    #     #     # out of road OR opponent car
    # def estimate_target_point(self, sensing_info):
    #     if len(sensing_info.track_forward_obstacles) != 0:
    #         area = []
    #         for obstacle in sensing_info.track_forward_obstacles:
    #             obs_dist = obstacle.get('dist')
    #             obs_to_middle = obstacle.get('to_middle')
                
    #             block_area(obs_dist, obs_to_middle, area)
    #     else:
    #         #전방 1구간의 도로 좌표 계산?

    def calc_target_to_middle(self, sensing_info):
        result = 0
        # self.estimate_target_point(sensing_info)
        return result

    def calc_target_point(self, sensing_info, target_to_middle):
        
            #direct_waypoint_dist = math.sqrt(math.pow(sensing_info.distance_to_way_points[1],2)-math.pow(sensing_info.to_middle, 2))
            #direct_waypoint_dist = (sensing_info.speed - self.adjusted_track_forward_angle[0] ) / 1

        target_dist = (self.speed_mps) * 3 * (max(90-5*self.adjusted_track_forward_angle[0],1))*0.01
        result = self.calc_angle(sensing_info.to_middle, 0, target_to_middle, target_dist)
        
        return result


    def calc_target_angle(self, sensing_info, target_to_middle):
        result = self.calc_target_point(sensing_info, target_to_middle) - sensing_info.moving_angle
        #result = self.calc_angle(sensing_info.to_middle, 0, target_point[0], target_point[1]) 

        return result

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

    def calc_target_road(self,sensing_info) :
        #position caculation : 좌표계산.
        #매 계산시마다 현 구간의 middle을 원점 O로 정한다. 이후 다음 구간의 좌표를 구한다. 즉, 좌표는 한 구간 내에서만 상대적으로 유의미하다.
        #theta가 주어졌을때 현 위치의 middle과 다음 위치의 middle을 연결하는 선이 중심각 theta인 호로 가정한다. 

        curve_datas = [] #[][][] 구간 / target_point / 좌표정보[pos_x, pos_y]
        curve_info = [] # 좌표데이터를 기반으로 생성한 판단용 데이터. [][][][] 구간 / start_point / target_point / datas[dist, angle, penalty value]

        # self.calc_road_position(sensing_info, curve_datas) #좌표 계산. curve_datas에 저장. 장애물 확인시 장애물 좌표 특이값 지정
        

        if self.DEBUG_LEVEL >= self.DEBUG_TRACE:
            print("### curve_datas ###")
            for matrix_data in curve_datas:
                print(matrix_data)
 
        # self.calc_road_info(sensing_info, curve_datas, curve_info) # 구간 정보 계산. curve_info에 저장. [][][][] : 구간번호 / 출발점 / 도착점 /  [dist, angle, priority]
        
        
        return curve_info

    def cosine_law(self, a, b, theta):
        # angle calculation impossible
        # estimate angle = 90 + theta/2
        result = math.sqrt(pow(a,2)+pow(b,2)-2*a*b*math.cos(theta))
        return result
    
    def calc_dist(self, start_x, start_y, target_x, target_y):
        if target_x >= self.MAX_VAL and target_y >= self.MAX_VAL:
            return self.MAX_VAL #dist max로 return 하여 선택되지 않도록.
        else:
            return round(math.sqrt(math.pow(target_x - start_x, 2) + math.pow(target_y - start_y, 2)),2)

    # def calc_angle(self, start_x, start_y, target_x, target_y):
    #     result = 0
    #     if (target_x - start_x)*(target_y - start_y) > 0:
    #         # positive angle 
    #         result = 1 * (90 - math.degrees(math.atan(abs((target_y-start_y)/(target_x-start_x)))))

    #     elif (target_x - start_x)*(target_y - start_y) < 0:
    #         #negative angle

    #         result = -1 * (90 - math.degrees(math.atan(abs((target_y-start_y)/(target_x-start_x)))))

    #     else :
    #         result = 0

    #     return result

    def mark_obstacles(self, obs_dist, obs_to_middle, interval, curve_datas, curve_matrix_index_adj, curve_matrix_index):
        pass
        # for index in range(curve_matrix_index_adj, curve_matrix_index+1):

        #     obs_left = obs_to_middle - 1.7 #여유수치를 준다.
        #     obs_right = obs_to_middle + 1.7

        #     blocked_left = 0
        #     blocked_left_flag = False
        #     blocked_right = self.target_point_level-1
        #     blocked_right_flag = False
        #     initial_val = -1*int(self.target_point_level/2)
            

        #     for point_index in range(self.target_point_level):

        #         if blocked_left_flag == False:
        #             if obs_left <= initial_val * interval:
        #                 blocked_left = max(point_index-1,0)
        #                 blocked_left_flag = True

        #         if blocked_right_flag == False:
        #             if obs_right <= initial_val * interval:
        #                 blocked_right = min(point_index, self.target_point_level-1)
        #                 blocked_right_flag = True
        #                 break

        #         initial_val += 1

        #     if self.DEBUG_LEVEL >= self.DEBUG_INFOS:
        #         print("index : {}, blocked_left : {}, blocked_right : {}".format(index, blocked_left, blocked_right))

        #     for block_index in range(blocked_left, blocked_right+1):

        #         curve_datas[index][block_index] = [self.MAX_VAL, self.MAX_VAL]

    
    # def calc_road_position(self, sensing_info, curve_datas):

    #     #angle adjust
    #     angle_index = 0
    #     for angle in sensing_info.track_forward_angles:
    #         sensing_info.track_forward_angles[angle_index] = angle - sensing_info.moving_angle
    #         angle_index +=1

    #     before_angle = 0 #angle값 보정을 위함. E.G. 구간정보가 3, 10일경우, 첫 구간의 angle은 3, 이후 구간은 angle은 상대적으로 7.

    #     for angle in sensing_info.track_forward_angles:

    #         real_angle = angle - before_angle
    #         before_angle = angle

    #         l = 10
    #         theta = abs(math.radians(real_angle)) # angle 값이 음수일경우 계산이 복잡하므로 전부 양수처리 후 좌표 y 대칭이동
    #         r = 0
            
    #         if theta == 0:
    #             r = 0
    #         else :
    #             r = round(l/theta,2)

    #         # 부채꼴로 가정시, middle_a -> middle_b로 이어지는 선분은 이등변 삼각형의 밑변
    #         bottom_angle = (180-abs(real_angle))/2
    #         bottom_line = self.cosine_law(r, r, theta)



    #         mid_x = 0
    #         mid_y = 0
    #         # theta에 따라 좌표 adjust시 사용할 변수.
    #         modif_x = 0
    #         modif_y = 0

    #         if real_angle > 0:
    #             mid_x = 1 * math.cos(math.radians(bottom_angle)) * bottom_line
    #             mid_y = math.sin(math.radians(bottom_angle)) * bottom_line
    #             modif_x = math.cos(theta) * self.interval
    #             modif_y = -1 * math.sin(theta) * self.interval
    #         elif real_angle <0:
    #             mid_x = -1 * math.cos(math.radians(bottom_angle)) * bottom_line
    #             mid_y = math.sin(math.radians(bottom_angle)) * bottom_line
    #             modif_x = math.cos(theta) * self.interval
    #             modif_y = 1 * math.sin(theta) * self.interval
    #         else:
    #             mid_x = 0
    #             mid_y = 10
    #             modif_x = self.interval
    #             modif_y = 0

    #         # 도로 n구간으로 구분.

    #         initial_val = -1*int(self.target_point_level/2)
    #         temp_list= []
    #         for index in range(self.target_point_level):
    #             temp_list.append([mid_x+initial_val*modif_x,mid_y+initial_val*modif_y])
    #             initial_val+= 1

    #         # curve_datas list에 각 포인트 좌표 저장
    #         curve_datas.append(temp_list)

    #         # 장애물 발견시, curve_datas에 저장한 구간별 target point에 표시. target point로 선정되는걸 막는다.
    #     if len(sensing_info.track_forward_obstacles) != 0:
    #         for obs_info in sensing_info.track_forward_obstacles:

    #             obs_dist = obs_info.get('dist')
    #             obs_to_middle = obs_info.get('to_middle')
                
    #             # obs_dist가 35일경우, 0~10 구간, 11~20구간, 21~30구간, 31~40구간에 속하므로, 4구간, 즉 3번 index로 접근한다.
    #             # speed가 높을경우, 바로 앞 구간만 판단시 회피시간이 부족하므로, 보다 앞 데이터 또한 막아준다.
    #             # speed*1/20만큼 index를 당긴다.
    #             # 예시
    #             # p p p p p p p
    #             # p p p p p p p
    #             # p p p p p O p <- 실제 obstacle 위치
    #             # p p p p p O p
    #             # p p p p p O p <- 속도에 따른 판단시간을 늘리기위해 Obstacle을 보다 가까운지점에도 있다고 마크한다.
    #             # p p p p p p p
                

    #             curve_matrix_index = min(max(int(obs_dist / 10), 0)+1, self.ROUTE_CALC_LEVEL-1)
    #             curve_matrix_index_adj = max(int(obs_dist / 10 - sensing_info.speed * 0.03 ), 0)

    #             self.mark_obstacles(obs_dist, obs_to_middle, self.interval, curve_datas, curve_matrix_index_adj, curve_matrix_index)

    # def calc_road_info(self, sensing_info, curve_datas, curve_info):
    #     area_index = 0 # 구간 index
    #     for area in curve_datas : 
    #         if area_index == 0 : # 현 vehicle 기준 전방 1구간
    #             start_pos = sensing_info.to_middle
    #             start_angle = sensing_info.moving_angle
                
    #             start_poses = []
    #             result = []

    #             target_index = 0
    #             before_max_val_flag = False
    #             for target_point in area:
    #                 calced_dist = self.calc_dist(start_pos, 0, target_point[0], target_point[1])
    #                 calced_angle = self.calc_angle(start_pos, 0, target_point[0], target_point[1]) - sensing_info.moving_angle
    #                 calced_penalty = self.calc_penalty(calced_dist, calced_angle, area_index, target_index)

    #                 #바로 전, 바로 후에 max_val(장애물)이 발생할 경우, penalty에 두배를 준다.
    #                 if before_max_val_flag == True:
    #                     calced_penalty *= 2
                    

    #                 result.append([calced_dist, calced_angle, calced_penalty])

    #                 #바로 전, 바로 후에 max_val(장애물)이 발생할 경우, penalty에 두배를 준다.
    #                 if calced_dist == self.MAX_VAL:
    #                     before_max_val_flag = True
    #                     result[max(target_index-1,0)][2] *= 2
    #                 else:
    #                     before_max_val_flag = False

    #                 target_index +=1

    #             start_poses.append(result)
    #             curve_info.append(start_poses)

    #         else : # 전체 구간. 1 구간당 5*5 25개 데이터 생성
    #             #start_positions = [-2*self.interval, -1*self.interval, 0, 1*self.interval, 2*self.interval]

    #             start_positions = []
    #             initial_val = -1*int(self.target_point_level/2)
    #             for index in range(self.target_point_level):
    #                 start_positions.append(initial_val*self.interval)
    #                 initial_val += 1

    #             start_poses = []
                
                
    #             for start_pos in start_positions:

    #                 result = []
    #                 target_index = 0
    #                 for target_point in area:
    #                     calced_dist = self.calc_dist(start_pos, 0, target_point[0], target_point[1])
    #                     calced_angle = self.calc_angle(start_pos, 0, target_point[0], target_point[1]) - sensing_info.moving_angle
    #                     calced_penalty = self.calc_penalty(calced_dist, calced_angle, area_index, target_index)

    #                     #바로 전, 바로 후에 max_val(장애물)이 발생할 경우, penalty에 두배를 준다.
    #                     if before_max_val_flag == True:
    #                         calced_penalty *= 2
                        

    #                     result.append([calced_dist, calced_angle, calced_penalty])

    #                     #바로 전, 바로 후에 max_val(장애물)이 발생할 경우, penalty에 두배를 준다.
    #                     if calced_dist == self.MAX_VAL:
    #                         before_max_val_flag = True
    #                         result[max(target_index-1,0)][2] *= 2
    #                     else:
    #                         before_max_val_flag = False

    #                     target_index += 1
                        

    #                 start_poses.append(result)
    #             curve_info.append(start_poses)        

    #         area_index += 1
    #         if area_index >= 10:
    #             break

    # def calc_penalty(self, calced_dist, calced_angle, area_index, target_index):
        #penalty 계산식
        #angle에 대한 penalty 식을 풀 경우, 주행시 변하는 angle에 의해 차량이 너울치는 현상 발생

    #     result = 1.0
    #     result *= calced_dist
    #     #result *= math.pow(1.2, abs(calced_angle)/50) 
    #     result *= math.pow(1.2,(self.ROUTE_CALC_LEVEL- area_index)) # 가까운 penalty에 가중치
    #     result *= (1 + abs(target_index - self.target_point_level/2)*0.2) # 도로 양쪽 끝 penalty에 가중치
    #     result *= 0.000001

    #     return  result
    #     #calced_dist * math.pow(3, abs(calced_angle)/50) * math.pow(1.2,(self.ROUTE_CALC_LEVEL- area_index)) * 0.0001 * (1 + abs(target_index - self.target_point_level/2)*0.3)
    #     # math.pow(1.2, abs(self.target_point_level/2 - target_index)) *

    # def calc_routes(self, sensing_info, curve_info):
        
    #     # 총 target_point_level개 seqence가 있는 list. 
    #     # target_point_level이 11일경우, 11개 지점에서 출발하는 route
    #     # self.route_calc_level 까지 계산한다.
 
    #     if self.DEBUG_LEVEL >= self.DEBUG_INFOS:
    #         print("### min_route_data calculation start ###")

    #     result = []
    #     for index in range(self.target_point_level):
    #         result.append(self.calc_route_penalty(curve_info, index, curve_info[0][0][index][0], curve_info[0][0][index][1], curve_info[0][0][index][2], [index]))

    #     return result

    # def calc_route_penalty(self, curve_info, src_index, distance, angle, penalty, chosen_sequence):        
    #     # curve_info : [area_index], [start_index], [target_index], [dist, angle, priority]

    #     #첫번째 loop일 경우 start_point는 하나뿐. 이에따른 도착점은 target_point_level개로 n개라 하자. 
    #     #그외  loop일경우, original_src_index에서 각 start_point로 가는 최단경우 n개. 이에따른 target_point은 n개로 n^2개 케이스.
    #     #이중 나머지 케이스는 버리고 최종 n개 타겟으로 가는 최선의 n개 케이스만 남긴다.

    #     chosen_sequence.append(distance)
    #     chosen_sequence.append(abs(angle))
    #     chosen_sequence.append(penalty)
    #     return_list = []
    #     output_list = [chosen_sequence]
        
    #     area_index = 1
    #     while True:

    #         if area_index >= self.ROUTE_CALC_LEVEL:
    #             break             

    #         for route_case in output_list:
    #             src_index = route_case[len(route_case)-4]
    #             temp_target_list = []

    #             target_index = 0
    #             for target_info in curve_info[area_index][src_index]:

    #                 distance_val = target_info[0]
    #                 angle_val = target_info[1]
    #                 penalty_val = target_info[2]
    #                 new_sequence = copy.deepcopy(route_case)
    #                 new_sequence.insert(area_index, target_index)
    #                 new_sequence[len(new_sequence)-3] += distance_val
    #                 new_sequence[len(new_sequence)-2] += abs(angle_val)
    #                 new_sequence[len(new_sequence)-1] += penalty_val
                    
    #                 if len(temp_target_list) <= target_index:
    #                     temp_target_list.insert(target_index, new_sequence)
    #                 else:
    #                     if temp_target_list[target_index][len(temp_target_list[target_index])-1] > new_sequence[len(new_sequence)-1]:
    #                         temp_target_list[target_index] = new_sequence
                    
    #                 target_index += 1

    #         output_list = copy.deepcopy(temp_target_list)
    #         area_index += 1

    #     min_total_penalty = self.MAX_VAL_100
    #     for route_case in output_list:
    #         if route_case[len(route_case)-1] < min_total_penalty:
    #             min_total_penalty = route_case[len(route_case)-1]
    #             return_list = route_case

    #     return return_list

    # def stabilize(self, car_controls, min_info):
    #     # 너울치는 현상을 줄이기 위해 만든 method
    #     last_index = len(self.decision_hist)-1
    #     if min_info[0] - self.decision_hist[last_index][0] == 0: ##마지막 선택 index와 현 선택 인덱스가 같다면
    #         car_controls.steering *= 0.8

        
    #     sum_steering = 0
    #     for infos in self.decision_hist:
    #         decision_index = infos[0]
    #         decision_throttle = infos[1]
    #         decision_steering = infos[2]
    #         decision_brake = infos[3]

    #         sum_steering += decision_steering

    #     #if abs(sum_steering) >= 30*self.HIST_LEVEL*0.015 and car_controls.steering*sum_steering <0: ##0.01 = steer 조정값
    #     #    car_controls.steering *= 2

    #     self.decision_hist.append([min_info[0], car_controls.throttle, car_controls.steering, car_controls.brake])

    #     if len(self.decision_hist)>= self.HIST_LEVEL:
    #         self.decision_hist.pop(0)
        

if __name__ == '__main__':
    client = DrivingClient()
    client.run()
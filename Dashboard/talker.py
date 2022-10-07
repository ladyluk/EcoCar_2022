#!/usr/bin/env python3.8

import random
import rospy
from std_msgs.msg import Float32

def talker():
    pubs = [
        #Battery - 0
        rospy.Publisher('HV_Bat_SOC', Float32, queue_size=10),
        rospy.Publisher('HV_Bat_Current', Float32, queue_size=10),
        rospy.Publisher('HV_Bus_V', Float32, queue_size=10),
        #Flowrate - 3
        rospy.Publisher('P0_Pump_Flow', Float32, queue_size=10),
        rospy.Publisher('ICE_Fan_Speed_PWM', Float32, queue_size=10),
        rospy.Publisher('P4_Pump_Flow', Float32, queue_size=10),
        rospy.Publisher('ICE_Coolant_Pump_PWM', Float32, queue_size=10),
        # 7
        rospy.Publisher('Lead_Vehicle_Bool', Float32, queue_size=10),
        rospy.Publisher('Lead_Vehicle_Long_Pos', Float32, queue_size=10),
        rospy.Publisher('Lead_Vehicle_Lat_Pos', Float32, queue_size=10),
        # 10
        rospy.Publisher('HV_Bat_Temp_C', Float32, queue_size=10),
        rospy.Publisher('P0_Motor_Temp_C', Float32, queue_size=10),
        rospy.Publisher('P0_Inverter_Temp_C', Float32, queue_size=10),
        rospy.Publisher('P4_Motor_Temp_C', Float32, queue_size=10),
        rospy.Publisher('P4_Inverter_Temp_C', Float32, queue_size=10),
        rospy.Publisher('ICE_Coolant_Temp_C', Float32, queue_size=10),
        rospy.Publisher('Trans_Coolant_Temp_C', Float32, queue_size=10),
        # 17
        rospy.Publisher('Distance_Envelope_Max', Float32, queue_size=10),
        rospy.Publisher('Distance_Envelope_Min', Float32, queue_size=10),
        # 19
        rospy.Publisher('ACC_State', Float32, queue_size=10),
        rospy.Publisher('ACC_Controller_Mode', Float32, queue_size=10),
        rospy.Publisher('Lead_Vehicle_TrackID', Float32, queue_size=10),
        #22
        rospy.Publisher('Camera_Fault', Float32,queue_size=10),
        rospy.Publisher('Radar_Fault', Float32,queue_size=10),
        rospy.Publisher('Tank_Fault', Float32,queue_size=10),
        rospy.Publisher('a_12V_Fault', Float32,queue_size=10),
        #26
        rospy.Publisher('P0_Fault', Float32,queue_size=10),
        rospy.Publisher('P4_Fault', Float32,queue_size=10),
        rospy.Publisher('ICE_Fault', Float32,queue_size=10),
        rospy.Publisher('HV_Fault', Float32,queue_size=10),
        rospy.Publisher('PCM_CAV_Fault', Float32,queue_size=10),
        rospy.Publisher('HV_Cooling_Fan', Float32,queue_size=10),
        #32
        rospy.Publisher('V2X_Signal', Float32,queue_size=10),
        rospy.Publisher('V2X_Remaining_Time', Float32,queue_size=10),
        rospy.Publisher('C2', Float32,queue_size=10),
        rospy.Publisher('C1', Float32,queue_size=10),
        rospy.Publisher('C0_R', Float32,queue_size=10),
        rospy.Publisher('C0_L', Float32,queue_size=10),
    ]
    rospy.init_node('talker', anonymous=True)
    rate=rospy.Rate(10)
    count=0

    HV_Bat_SOC = 0
    HV_Bat_Current = 0
    HV_Bus_V = 0
    
    P0_Pump_Flow = 0
    ICE_Fan_Speed_PWM = 0
    P4_Pump_Flow = 0
    ICE_Coolant_Pump_PWM = 0
    
    Lead_Vehicle_Present = 1
    Lead_Vehicle_Long_Dist = 0
    Lead_Vehicle_Lat_Dist = 0
    
    HV_Bat_Temp_C = 0
    ACC_State = 0
    ACC_Controller_Mode = 0
    Lead_Vehicle_TrackID = 0

    Camera_Fault = 0
    Radar_Fault = 0
    Tank_Fault = 0
    a_12V_Fault = 0

    P0_Fault = 0
    P4_Fault = 0
    ICE_Fault = 0
    HV_Fault = 0
    PCM_CAV_Fault = 0
    HV_Cooling_Fan = 0

    V2X_Signal = 0
    V2X_Remaining_Time = 5
    
    while not rospy.is_shutdown():
        HV_Bat_SOC = (HV_Bat_SOC+random.uniform(1, 3))%100
        HV_Bat_Current =random.uniform(0,500)
        HV_Bus_V = random.uniform(0,400)

        P0_Pump_Flow = random.uniform(0,100)
        ICE_Fan_Speed_PWM = random.uniform(0,100)
        P4_Pump_Flow = random.uniform(0,100)
        ICE_Coolant_Pump_PWM = random.uniform(0,100)
        HV_Cooling_Fan = random.randint(0,1)
        
        # Lead_Vehicle_Long_Dist+=random.uniform(-10,10)
        Lead_Vehicle_Lat_Dist+=random.uniform(-10,10)
        
        # Lead_Vehicle_Long_Dist=max(0, min(255, Lead_Vehicle_Long_Dist))
        Lead_Vehicle_Lat_Dist=max(-30, min(30, Lead_Vehicle_Lat_Dist))
        Lead_Vehicle_Long_Dist=(Lead_Vehicle_Long_Dist+random.uniform(1,3))%120
        
        if count%10==0:
            HV_Bat_Temp_C=(HV_Bat_Temp_C+random.uniform(1,3))%100
        
        Camera_Fault = random.randint(0,3)
        Radar_Fault = random.randint(0,2)
        Tank_Fault = random.randint(0,3)
        a_12V_Fault = random.randint(0,1)
        
        P0_Fault = 2**(random.randint(0,9))
        P4_Fault = 2**(random.randint(0,8))
        ICE_Fault = 2**(random.randint(0,5))
        HV_Fault = 2**(random.randint(0,4))
        PCM_CAV_Fault = 2**(random.randint(0,5))

        V2X_Remaining_Time -= 0.1
        if(V2X_Remaining_Time<0):
            V2X_Remaining_Time=5

        pubs[0].publish(HV_Bat_SOC)
        pubs[1].publish(HV_Bat_Current)
        pubs[2].publish(HV_Bus_V)
        pubs[3].publish(P0_Pump_Flow)
        pubs[4].publish(ICE_Fan_Speed_PWM)
        pubs[5].publish(P4_Pump_Flow)
        pubs[6].publish(ICE_Coolant_Pump_PWM)
        if count%50==0:
            Lead_Vehicle_Present = 1-Lead_Vehicle_Present
            pubs[7].publish(Lead_Vehicle_Present)
        pubs[8].publish(Lead_Vehicle_Long_Dist)
        pubs[9].publish(Lead_Vehicle_Lat_Dist)
        for i in range(10,17):
            pubs[i].publish(HV_Bat_Temp_C-20)
        if count%10==0:
            pubs[17].publish(random.uniform(70,90))
            pubs[18].publish(random.uniform(20,50))
        if count%20==0:
            pubs[19].publish(ACC_State)
            pubs[20].publish(ACC_Controller_Mode)
            pubs[21].publish(Lead_Vehicle_TrackID)
            ACC_State = (ACC_State+1)%4
            ACC_Controller_Mode = (ACC_Controller_Mode+1)%6
            Lead_Vehicle_TrackID+=1
        
        pubs[22].publish(Camera_Fault)
        pubs[23].publish(Radar_Fault)
        pubs[24].publish(Tank_Fault)
        pubs[25].publish(a_12V_Fault)
        
        pubs[26].publish(P0_Fault)
        pubs[27].publish(P4_Fault)
        pubs[28].publish(ICE_Fault)
        pubs[29].publish(HV_Fault)
        pubs[30].publish(PCM_CAV_Fault)
        pubs[31].publish(HV_Cooling_Fan)

        if count%51==0:
            pubs[32].publish(V2X_Signal)
            V2X_Signal=(V2X_Signal+1)%4

        pubs[33].publish(V2X_Remaining_Time)

        pubs[34].publish(0.001)
        pubs[35].publish(0.1)
        pubs[36].publish(-2) # right
        pubs[37].publish(2)  # left


        count+=1
        rate.sleep()

if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
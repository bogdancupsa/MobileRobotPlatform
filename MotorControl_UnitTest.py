import time
import unittest

turn_value = 0
stop_limit = 25
right_motor_value      = 0
left_motor_value       = 0
direction_left_front   = 0
direction_right_front  = 0
direction_left_rear    = 0
direction_right_rear   = 0

def MotorControlTask(value_for_front_distance_sensor):
    result = "NONE"
    i = 0
    j = 0
    k = 0
    for x in value_for_front_distance_sensor:
        if x > stop_limit:
            turn_value = 2 #GO
            right_motor_value = 1000 #50% fill PWM factor
            left_motor_value = 1000 #50% fill PWM factor
            direction_left_front = 0 #FORWARD
            direction_right_front = 0 #FORWARD
            direction_left_rear = 0 #FORWARD
            direction_right_rear = 0 #FORWARD
            if i == 0:
                if result == "NONE":
                    result = "GO"
                else:   
                    result += "_GO"
                i+=1
            j = 0
            k = 0
        elif x > 0 and x < stop_limit:
            turn_value = 1 #TURN_RIGHT
            right_motor_value = 1200 #60% fill PWM factor
            left_motor_value = 1200 #60% fill PWM factor
            direction_left_front = 0 #FORWARD
            direction_right_front = 1 #BACKWARDS
            direction_left_rear = 0 #FORWARD
            direction_right_rear = 1 #BACKWARDS 
            if j == 0:
                if result == "NONE":
                    result = "TURN_RIGHT"
                else:   
                    result += "_TURN_RIGHT"
                j+=1    
            i = 0
            k = 0  
        else:
            turn_value = 0 #TURN_LEFT
            right_motor_value = 1200 #60% fill PWM factor
            left_motor_value = 1200 #60% fill PWM factor
            direction_left_front = 1 #BACKWARDS
            direction_right_front = 0 #FORWARD
            direction_left_rear = 1 #BACKWARDS
            direction_right_rear = 0 #FORWARD     
            if k == 0:
                if result == "NONE":
                    result = "TURN_LEFT"
                else:    
                    result += "_TURN_LEFT"
                k+=1  
            i = 0
            j = 0
        print(turn_value, right_motor_value, left_motor_value, direction_left_front, direction_right_front, direction_left_rear, direction_right_rear)
        time.sleep(0.1)
    return result


#Test cases
TURN_RIGHT_TEST_VECTOR = [1,9,2,5,8,23,14,11,20,17]
TURN_LEFT_TEST_VECTOR = [0,0,0,0,0,0,0,0,0,0]
GO_TEST_VECTOR = [26,26,26,26,26,26,26,26,26,26]
GO_TURN_RIGHT_TEST_VECTOR = [26,26,26,26,26,10,23,12,9,1]
GO_TURN_RIGHT_GO_TURN_LEFT_GO_TEST_VECTOR = [26,26,10,20,26,26,0,0,26,26]

class TestSum(unittest.TestCase):

    def testcase_1(self):
        self.assertEqual(MotorControlTask(TURN_RIGHT_TEST_VECTOR), "TURN_RIGHT")

    def testcase_2(self):
       self.assertEqual(MotorControlTask(TURN_LEFT_TEST_VECTOR), "TURN_LEFT")

    def testcase_3(self):
       self.assertEqual(MotorControlTask(GO_TURN_RIGHT_TEST_VECTOR), "GO_TURN_RIGHT")

    def testcase_4(self):
       self.assertEqual(MotorControlTask(GO_TEST_VECTOR), "GO")

    def testcase_5(self):
       self.assertEqual(MotorControlTask(GO_TURN_RIGHT_GO_TURN_LEFT_GO_TEST_VECTOR), "GO_TURN_RIGHT_GO_TURN_LEFT_GO")

if __name__ == '__main__':
    unittest.main()
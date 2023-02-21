import numpy as np
import cv2 as cv
import math
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import PositionTarget

video = cv.VideoCapture(0)
current_state = State()
att_msg = PositionTarget()

def state_cb(msg):
    global current_state
    current_state = msg
    
if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 5

    count = 0
    count1 = 0

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()
        # Batas
            _, frame = video.read()
            frame = cv.flip(frame, 1)
            kernel = np.ones((3,3),np.uint8)
            roi = frame[200:400, 400:600]

            cv.rectangle(frame,(400,200),(600,400),(0,255,0),0)  
            hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

            lower = np.array([0, 20, 70])
            upper = np.array([20, 255, 255])

            mask = cv.inRange(hsv, lower, upper)
            mask = cv.dilate(mask,kernel,iterations = 4)
            mask = cv.GaussianBlur(mask,(5,5),100) 

            contours, hierarchy = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)

            cnt = max(contours, key = lambda x: cv.contourArea(x))

            epsilon = 0.0005*cv.arcLength(cnt,True)

            approx= cv.approxPolyDP(cnt,epsilon,True)

            hull = cv.convexHull(cnt)

            areahull = cv.contourArea(hull)
            areacnt = cv.contourArea(cnt)
            arearatio=((areahull-areacnt)/areacnt)*100

            hull = cv.convexHull(approx, returnPoints=False)
            defects = cv.convexityDefects(approx, hull)

            l = 0

            for i in range(defects.shape[0]):
                s,e,f,d = defects[i,0]
                start = tuple(approx[s][0])
                end = tuple(approx[e][0])
                far = tuple(approx[f][0])
                pt= (100,180)


                # find length of all sides of triangle
                a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
                b = math.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
                c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
                s = (a+b+c)/2
                ar = math.sqrt(s*(s-a)*(s-b)*(s-c))

                #distance between point and convex hull
                d=(2*ar)/a

                # apply cosine rule here
                angle = math.acos((b**2 + c**2 - a**2)/(2*b*c)) * 57


                # ignore angles > 90 and ignore points very close to convex hull(they generally come due to noise)
                if angle <= 90 and d>30:
                    l += 1
                    cv.circle(roi, far, 3, [255,0,0], -1)

                #draw lines around hand
                cv.line(roi,start, end, [0,255,0], 2)

            l+=1
            if l==2:
                if count >= 50 and count < 100:
                    pose.pose.orientation.z = 180
                    pose.pose.orientation.y = -90
                    pose.pose.position.x = 10
                    pose.pose.position.y = 0
                elif count >= 100 and count < 150:
                    pose.pose.orientation.x = 90
                    pose.pose.orientation.z = -90
                    pose.pose.position.x = 10
                    pose.pose.position.y = 8
                elif count >= 150 and count < 200:
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.y = 0
                    pose.pose.orientation.x = 0
                    pose.pose.position.x = 0
                    pose.pose.position.y = 8
                elif count >= 200 and count < 250:
                    pose.pose.orientation.x = 90
                    pose.pose.orientation.z = 90 
                    pose.pose.position.x = 0
                    pose.pose.position.y = 0
                elif count >= 250 and count < 300:
                    pose.pose.orientation.x = 90
                    pose.pose.orientation.z = 80 
                    pose.pose.position.x = 5
                    pose.pose.position.y = -abs(5)
                elif count >= 300 and count < 350:
                    pose.pose.orientation.x = 90
                    pose.pose.orientation.z = -90
                    pose.pose.position.x = 10
                    pose.pose.position.y = 0
                elif count >= 350 and count < 400:
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.y = 0
                    pose.pose.orientation.x = 0
                    pose.pose.position.x = 0
                    pose.pose.position.y = 0
                elif count > 400:
                    count = 0
                count += 1
                rospy.loginfo("Make a House " + str(count))
            elif l == 3:
                pose.pose.orientation.z = 1
                pose.pose.orientation.w = 1
                pose.pose.orientation.y = 0
                pose.pose.orientation.x = 0
                count1 += 1
                rospy.loginfo("Rotation 90 degrees")
            elif l == 4:
                pose.pose.orientation.z = 1
                pose.pose.orientation.y = 1
                pose.pose.orientation.w = 0
                pose.pose.orientation.x = 0
                rospy.loginfo("Rotatioin 180 degrees")
            elif l==5:
                if count1 > 0 and count1 <= 15:
                    pose.pose.orientation.z = 1
                    pose.pose.orientation.y = 1
                elif count1 > 15 and count1 <= 20:
                    pose.pose.orientation.z = 1
                    pose.pose.orientation.w = 1
                    pose.pose.orientation.y = 0
                    pose.pose.orientation.x = 0
                elif count1 > 20 and count1 <= 30:
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.y = 0
                elif count1 > 30:
                    count1 = 0
                count1 += 1
                rospy.loginfo("Spin " + str(count1))
            else:
                rospy.loginfo("Waiting for Command")
                count = 0
                count1 = 0

                pose.pose.orientation.x = 0
                pose.pose.orientation.y = 0
                pose.pose.orientation.z = 0
                pose.pose.position.x = 0
                pose.pose.position.y = 0

            cv.imshow("WEBCAM", frame)

            key = cv.waitKey(1)
            if key == 27:
                    break

        # Batas

        local_pos_pub.publish(pose)

        rate.sleep()
    

video.release()
cv.destroyAllWindows()

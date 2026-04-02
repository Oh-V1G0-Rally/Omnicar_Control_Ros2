#!/usr/bin/env python3
import rospy
from sdpo_ros_interfaces_hw.msg import mot_ref [cite: 8]

def run_test():
    rospy.init_node('timed_motor_test')
    # Il topic deve includere il namespace 'unnamed_robot'
    pub = rospy.Publisher('/unnamed_robot/motors_ref', mot_ref, queue_size=1)
    
    rate = rospy.Rate(20) # 20Hz per soddisfare il watchdog di 0.2s
    
    # --- FASE 1: Avvio a velocità 5.0 ---
    rospy.loginfo("Inizio test: Velocità 5.0 per 10 secondi")
    start_time = rospy.get_time()
    while rospy.get_time() - start_time < 10.0 and not rospy.is_shutdown():
        msg = mot_ref()
        msg.angular_speed_ref = [5.0, 5.0, 5.0, 5.0] [cite: 8]
        pub.publish(msg)
        rate.sleep()

    # --- FASE 2: Stop (Velocità 0) ---
    rospy.loginfo("Test completato: Invio comando di STOP")
    for _ in range(10): # Invia lo zero per mezzo secondo per sicurezza
        msg = mot_ref()
        msg.angular_speed_ref = [0.0, 0.0, 0.0, 0.0] [cite: 8]
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        run_test()
    except rospy.ROSInterruptException:
        pass

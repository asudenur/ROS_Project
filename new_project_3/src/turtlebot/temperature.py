#!/usr/bin/env python3

import rospy
import requests
from std_msgs.msg import Float32

def get_temperature(api_key):
    api_url = f'http://api.openweathermap.org/data/2.5/weather?q=Konya&appid={api_key}&units=metric'
    
    try:
        response = requests.get(api_url)
        
        if response.status_code == 200:
            temperature_data = response.json()
            temperature = temperature_data['main']['temp']
            return temperature
        else:
            rospy.logwarn(f"Could not fetch temperature data. Status code: {response.status_code}")
            rospy.logwarn("Response: %s", response.json())  # Yanıtın detaylarını yazdır
            return None
    except requests.exceptions.RequestException as e:
        rospy.logerr(f"Error fetching temperature data: {e}")
        return None

def temperature_publisher(api_key):
    pub = rospy.Publisher('konya_temperature', Float32, queue_size=10)
    rospy.init_node('temperature', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        temperature = get_temperature(api_key)
        if temperature is not None:
            pub.publish(temperature)
            rospy.loginfo(f"Konya'nın anlık sıcaklığı: {temperature} °C")
        else:
            rospy.logwarn("Sıcaklık bilgisi alınamadı.")
        rate.sleep()

if __name__ == '__main__':
    API_KEY = 'c2b56bcc596e49a07b05cac63f6b32c7'  # API anahtarınızı buraya girin
    try:
        temperature_publisher(API_KEY)
    except rospy.ROSInterruptException:
        pass


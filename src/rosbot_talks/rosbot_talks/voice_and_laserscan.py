import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.duration import Duration
import re
import pygame
import elevenlabs
from elevenlabs import voices, generate
import openai
import speech_recognition as sr
import math
import io
from dotenv import load_dotenv
import os
from rclpy.qos import QoSProfile
from time import sleep

# Load environment variables
load_dotenv()
api_key = os.getenv("API_KEY")
api_key2 = os.getenv("API_KEY_Eleven")

elevenlabs.set_api_key(api_key2)
openai.api_key = api_key

# Global variables
flag = True
motion_publisher = None
text_publisher = None
node = None
lidar_data = None

# TTS and playing audio
def play_sound(message):
    pygame.init()
    pygame.mixer.init()

    try:
        audio = elevenlabs.generate(
            text=message,
            voice="Brian",
            model='eleven_turbo_v2_5'
        )
        sound = pygame.mixer.Sound(io.BytesIO(audio))
        sound.play()
        while pygame.mixer.get_busy():
            pygame.time.Clock().tick(10)

    except pygame.error as e:
        print("Cannot play the audio:", e)

    finally:
        pygame.mixer.quit()
        pygame.quit()

def interpret_command_with_chatgpt(command):
    """Use GPT to interpret and correct a voice command."""
    prompt_text = f"""Perform the following operations on the provided {command} given by a human that involves direction and distance:
        You should determine the direction as straight, left, right, or stop ONLY.
        Output the corrected command ONLY. No flavour text.
        If the command is "turn around" or any synonym of it, the output should be "turn left 360 degrees." 
        However, if the command specifies a direction such as clockwise or anti-clockwise, the output should be "turn right 360 degrees" and "turn left 360 degrees" respectively.
        If the direction is like backwards, specify the direction as 'straight' and convert the distance to negative.
        In command mentions anything like diagonal movement, the response should be 'turn left 15 degrees and move straight specified units'.
        Please make sure to rectify any potential spelling errors or homophone mistakes.
    """
    try:
        response = openai.ChatCompletion.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": "You are a helpful assistant."},
                {"role": "user", "content": prompt_text}
            ]
        )
        return response['choices'][0]['message']['content'].strip()
    except Exception as e:
        print(f"Error with OpenAI: {e}")
        return "Error processing command"

def speech_to_text_callback():
    global flag, text_publisher
    if flag:
        create_text("Generate a short simple salutation eager for the human to direct you.")
    else:
        create_text("Generate one sentence to show that you are ready to listen for next direction")

    flag = False
    recognizer = sr.Recognizer()

    try:
        with sr.Microphone() as source:
            print("Listening now: ")
            audio = recognizer.listen(source, timeout=5)
            print("Stopped Listening")
            play_sound("Okay, processing")
            text = recognizer.recognize_google(audio, show_all=True)
            print(str(text))

        alternative_list = text.get('alternative', [])

        # Iterating to find text with numeric digits
        selected_text = ""
        for item in alternative_list:
            transcript = item.get('transcript', '')
            if any(char.isdigit() for char in transcript):
                selected_text = transcript
                break

        # If no text with numeric digits found, select the first one
        if selected_text == '' and alternative_list:
            selected_text = alternative_list[0].get('transcript', '')

        selected_text = interpret_command_with_chatgpt(selected_text)
        print("Selected Text: " + selected_text)
        msg = String()
        msg.data = selected_text
        text_publisher.publish(msg)

    except Exception as e:
        print('Error: ' + str(e))
        speech_to_text_callback()

def create_text(prompt):
    """Generate textual responses using GPT."""
    try:
        completion = openai.ChatCompletion.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": "You are a bot."},
                {"role": "user", "content": prompt}
            ]
        )
        return completion.choices[0].message.content
    except Exception as e:
        print(f"Error with OpenAI: {e}")
        return "Error generating response"

def process_voice_command(msg):
    global text_subscriber
    text = str(msg).lower()
    digit_match = re.search(r'(-?[0-9]+)\b(?!\s*(degrees|d|Degrees))', text) #digit match from the image
    degrees_match = re.search(r'([0-9]+)\s*(degrees|d|Degrees)?\b', text) # degrees match from the image

    if degrees_match:
        degrees = int(degrees_match.group(1))
        print(degrees)
    else:
        degrees = 45 

    if digit_match:
        distance_to_travel = int(digit_match.group(1))  
    else:
        distance_to_travel = 0

    if not check_lidar_data(distance_to_travel):
        play_sound("There is an obstacle in the way. Movement cannot be completed.")
        return

    msg = msg.tolower().split()

    words = text.split()
    for word in words:
        if "left" in word:
            print("Command: Left")
            print(f"Degrees: {degrees}")
            turn_by_angle(degrees, 0.5)
            move_linear(distance_to_travel, 0.2)

        if "right" in word:
            print("Command: Right")
            turn_by_angle(-degrees, 0.5)
            move_linear(distance_to_travel, 0.2)

        if "straight" in word:
            print("Command: Straight")
            move_linear(distance_to_travel, 0.2)

    speech_to_text_callback()

def check_lidar_data(distance):
    global lidar_data
    if lidar_data is None:
        play_sound("No LiDAR data available.")
        return False

    # Assume lidar_data is a LaserScan message with ranges attribute
    min_distance = min(lidar_data.ranges)
    if min_distance < distance / 100.0:  # Convert distance to meters
        play_sound(f"Obstacle detected at {min_distance} meters, which is less than {distance / 100.0} meters.")
        return False

    return True

def lidar_callback(msg):
    global lidar_data
    lidar_data = msg

def turn_by_angle(angle_degrees, angular_velocity):
    global motion_publisher, node
    angle_radians = math.radians(angle_degrees)

    motion_command = Twist()
    motion_command.angular.z = angular_velocity if angle_degrees > 0 else -angular_velocity

    turn_duration_sec = abs(angle_radians / angular_velocity)

    print(f"Turning {'left' if angle_degrees > 0 else 'right'} by {abs(angle_degrees)} degrees")

    end_time = node.get_clock().now() + Duration(seconds=turn_duration_sec)
    while node.get_clock().now() < end_time:
        motion_publisher.publish(motion_command)
        rclpy.spin_once(node, timeout_sec=0.1)

    motion_command.angular.z = 0.0
    motion_publisher.publish(motion_command)
    print("Turn complete.")

def move_linear(distance_cm, linear_velocity):
    global motion_publisher, node
    if distance_cm < 0:
        distance_cm = abs(distance_cm)
        linear_velocity *= -1
    distance_m = distance_cm / 100 

    motion_command = Twist()
    motion_command.linear.x = linear_velocity

    move_duration_sec = abs(distance_m / linear_velocity)

    print(f"Moving forward {distance_cm} cm for {move_duration_sec} seconds.")

    end_time = node.get_clock().now() + Duration(seconds=move_duration_sec)
    while node.get_clock().now() < end_time:
        motion_publisher.publish(motion_command)
        rclpy.spin_once(node, timeout_sec=0.1)

    motion_command.linear.x = 0.0
    motion_publisher.publish(motion_command)
    print("Movement complete.")
    
def main(args=None):
    global text_publisher, motion_publisher, text_subscriber, node
    rclpy.init(args=args)
    node = Node('voice_control_node')
    text_publisher = node.create_publisher(String,'/recognized_text',1)  # Publishing text
    qos_profile = QoSProfile(depth=10)
    text_subscriber = node.create_subscription(String, '/recognized_text', process_voice_command, qos_profile) # custom topic
    lidar_subscriber = node.create_subscription(LaserScan, '/scan', lidar_callback, qos_profile)  # LiDAR topic
    motion_publisher = node.create_publisher(Twist, '/cmd_vel', 1)  # Publishing movement commands to this topic
    speech_to_text_callback()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()

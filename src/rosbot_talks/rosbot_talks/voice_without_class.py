import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
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

def play_sound(text, voice="Bella"):
    pygame.init()
    pygame.mixer.init()
    
    audio = generate(text)
    try:
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
    try:
        prompt_text = f"""Perform the following operations on the provided {command} given by a human that involves direction and distance:
            Determine the direction as straight, left, right, or stop.
            Present your response ONLY in the format (direction by distance).
            If no distance is specified in the command, indicate the distance as 0cm.
            For directions like about turn, state the angle as 180.
            If the direction is backward, specify the direction as 'straight' with a negative distance.
            In the case of diagonal movement, the response should be 'turn left 15 degrees and move straight specified units'.
            If the direction is turn around, the output should be turn left 360 degrees.
            Please make sure to rectify any potential spelling errors or homophone mistakes."""

        completion = openai.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": "You are a bot."},
            {"role": "user", "content": prompt_text}
        ]
        )
        return completion.choices[0].message.content
    
    except Exception as e:
        print(f"An error occurred: {e}")
        return "Error processing command"

def speech_to_text_callback():
    global flag, text_publisher
    if flag:
        create_text("Generate a short simple salutation eager for the human to direct you")
    else:
        create_text("Generate one sentence to show that you are ready to listen for next direction")

    flag = False
    recognizer = sr.Recognizer()

    try:
        with sr.Microphone() as source:
            print("Listening now: ")
            audio = recognizer.listen(source, timeout=3)
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
        text_publisher.publish(String(data=selected_text))
        process_voice_command(selected_text)

    except Exception as e:
        print('Error: ' + str(e))
        speech_to_text_callback()

def create_text(prompting):
    completion = openai.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": "You are a bot."},
            {"role": "user", "content": prompting}
        ]
    )
    response = completion.choices[0].message.content
    play_sound(response)

def process_voice_command(msg):
    text = str(msg).lower()
    digit_match = re.search(r'\s([0-9]+)', text)

    if digit_match:
        distance_to_travel = int(digit_match.group(1))  
    else:
        distance_to_travel = 0

    print(f"Linear Value: {distance_to_travel}")
    
    if "degree" in text:
        digit_match = re.search(r'\s([0-9]+)', text)
        degrees = int(digit_match.group(1))
        turn_by_angle(degrees, 0.5)

    if "left" in text:
        print("Command: Left")
        turn_by_angle(45, 0.5)
        move_linear(distance_to_travel, 0.2)

    elif "right" in text:
        print("Command: Right")
        turn_by_angle(-45, 0.5)
        move_linear(distance_to_travel, 0.2)

    elif "straight" in text:
        print("Command: Straight")
        move_linear(distance_to_travel, 0.2)

    else:
        print("Command: Stop")
        move_linear(0, 0)  

    speech_to_text_callback()

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
    speech_to_text_callback()

def move_linear(distance_cm, linear_velocity):
    global motion_publisher, node
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
    # motion_publisher.publish(motion_command)
    print("Movement complete.")
    
    return

def main(args=None):
    global text_publisher, motion_publisher, text_subscriber, node
    rclpy.init(args=args)
    node = Node('speech_to_text_node')
    text_publisher = node.create_publisher(String,'/recognized_text',1)  # Publishing text
    qos_profile = QoSProfile(depth=10)
    text_subscriber = node.create_subscription(String, '/recognized_text', process_voice_command, qos_profile)
    motion_publisher = node.create_publisher(  Twist, '/cmd_vel', 1)  # Publishing movement commands
    speech_to_text_callback()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()
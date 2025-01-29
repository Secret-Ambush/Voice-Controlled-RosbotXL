import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.duration import Duration
import re
import pygame
import speech_recognition as sr
import math
import io
from dotenv import load_dotenv
import os
from rclpy.qos import QoSProfile
from time import sleep
import ollama
import io
from melo.api import TTS

# Load environment variables
load_dotenv('~/ros2_ws/src/voice_bot/.env')

api_key2 = os.getenv("API_KEY_Eleven")

# Global variables
flag = True
motion_publisher = None
text_publisher = None
node = None
lidar_data = None

# Speed is adjustable
speed = 1.0

# CPU is sufficient for real-time inference.
# You can set it manually to 'cpu' or 'cuda' or 'cuda:0' or 'mps'
device = 'auto'  # Will automatically use GPU if available

# English 
model = TTS(language='EN', device=device)
speaker_ids = model.hps.data.spk2id

def play_sound(text):
    pygame.init()
    pygame.mixer.init()
    
    try:
        audio_data = model.tts(text, speaker_ids['EN-BR'], speed=speed)
        sound = pygame.mixer.Sound(io.BytesIO(audio_data))
        sound.play()
        while pygame.mixer.get_busy():
            pygame.time.Clock().tick(10)
    except pygame.error as e:
        print("Cannot play the audio:", e)
    finally:
        pygame.mixer.quit()
        pygame.quit()

def interpret_command_with_chatgpt(command):
    """Use Llama 3.2 to interpret and correct a voice command."""
    prompt_text = f"""Perform the following operations on the provided '{command}' given by a human that involves direction and distance:
        - Determine the direction as 'straight', 'left', 'right', or 'stop' ONLY.
        - Output the corrected command ONLY. No additional text.
        - If the command is "turn around", output "turn left 360 degrees."
        - If the direction is specified as clockwise or anti-clockwise, output "turn right 360 degrees" or "turn left 360 degrees" respectively.
        - If the direction is 'backwards', specify it as 'straight' and convert the distance to negative.
        - If the command mentions diagonal movement, convert it to 'turn left 15 degrees and move straight specified units'.
        - Correct spelling and homophone mistakes.
    """
    try:
        response = ollama.chat(model="meta/llama3", messages=[{"role": "user", "content": prompt_text}])
        return response["message"]["content"].strip()
    except Exception as e:
        print(f"Error with Ollama (Llama 3.2): {e}")
        return "Error processing command"

def speech_to_text_callback():
    global flag, text_publisher
    if flag:
        create_text("Generate a short simple salutation eager for the human to direct you.")
    else:
        create_text("Generate one sentence to show that you are ready to listen for the next direction.")

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

        if selected_text == '' and alternative_list:
            selected_text = alternative_list[0].get('transcript', '')

        selected_text = interpret_command_with_chatgpt(selected_text)
        print("Selected Text: " + selected_text)
        msg = String()
        msg.data = selected_text
        text_publisher.publish(msg)
        process_voice_command(msg)

    except Exception as e:
        print('Error: ' + str(e))
        speech_to_text_callback()

def create_text(prompt):
    """Generate textual responses using Deepseek-r1."""
    try:
        response = ollama.chat(model="deepseek-r1", messages=[{"role": "user", "content": prompt}])
        message_to_speak = response["message"]["content"]
        play_sound(message_to_speak)
    except Exception as e:
        print(f"Error with Deepseek-R1: {e}")
        return "Error generating response"

def process_voice_command(msg):
    global text_subscriber
    text = str(msg).lower()
    digit_match = re.search(r'(-?[0-9]+)\b(?!\s*(degrees|d|Degrees))', text)
    degrees_match = re.search(r'([0-9]+)\s*(degrees|d|Degrees)?\b', text)

    degrees = int(degrees_match.group(1)) if degrees_match else 45
    distance_to_travel = int(digit_match.group(1)) if digit_match else 0

    words = text.split()
    for word in words:
        if "left" in word:
            print("Command: Left")
            print(f"Degrees: {degrees}")
            turn_by_angle(degrees, 0.5)
            move_linear(distance_to_travel, 0.2)
        elif "right" in word:
            print("Command: Right")
            print(f"Degrees: {degrees}")
            turn_by_angle(-degrees, 0.5)
            move_linear(distance_to_travel, 0.2)
        elif "straight" in word:
            print("Command: Straight")
            move_linear(distance_to_travel, 0.2)

    speech_to_text_callback()

def main(args=None):
    global text_publisher, motion_publisher, text_subscriber, node
    rclpy.init(args=args)
    node = Node('voice_control_node')
    text_publisher = node.create_publisher(String, '/recognized_text', 1)
    qos_profile = QoSProfile(depth=10)
    text_subscriber = node.create_subscription(String, '/recognized_text', process_voice_command, qos_profile)
    lidar_subscriber = node.create_subscription(LaserScan, '/scan', lidar_callback, qos_profile)
    motion_publisher = node.create_publisher(Twist, '/cmd_vel', 1)
    speech_to_text_callback()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
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
import argparse
from queue import Queue
import whisper
from time import sleep
from datetime import datetime, timedelta
import numpy as np
import torch

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
            You should determine the direction as straight, left, right, or stop ONLY.
            Output the corrected command ONLY. No flavour text.
            If the command is "turn around" or any synonym of it, the output should be "turn left 360 degrees." 
            However, if the command specifies a direction such as clockwise or anti-clockwise, the output should be "turn right 360 degrees" and "turn left 360 degrees" respectively.
            If the direction is like backwards, specify the direction as 'straight' and convert the distance to negative.
            In command mentions anything like diagonal movement, the response should be 'turn left 15 degrees and move straight specified units'.
            Please make sure to rectify any potential spelling errors or homophone mistakes.
        """

        # gpt4o could be included
        response = openai.ChatCompletion.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": "You are a helpful assistant."},
                {"role": "user", "content": prompt_text}
            ]
        )
        return response['choices'][0]['message']['content'].strip()
    
    except Exception as e:
        print(f"An error occurred: {e}")
        return "Error processing command"

def speech_to_text_callback():
    global flag, text_publisher
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="medium", help="Model to use",
                        choices=["tiny", "base", "small", "medium", "large"])
    parser.add_argument("--energy_threshold", default=1000,
                        help="Energy level for mic to detect.", type=int)
    parser.add_argument("--record_timeout", default=2,
                        help="How real time the recording is in seconds.", type=float)
    parser.add_argument("--phrase_timeout", default=3,
                        help="How much empty space between recordings before we "
                            "consider it a new line in the transcription.", type=float)
    args = parser.parse_args()
    
    phrase_time = None
    data_queue = Queue()
    recorder = sr.Recognizer()
    recorder.energy_threshold = args.energy_threshold
    recorder.dynamic_energy_threshold = False
    source = sr.Microphone(sample_rate=16000)
    
    
    model = args.model
    if args.model != "large":
        model = model + ".en"
    audio_model = whisper.load_model(model)

    record_timeout = args.record_timeout
    phrase_timeout = args.phrase_timeout

    transcription = ['']

    with source:
        recorder.adjust_for_ambient_noise(source)
        
    def record_callback(_, audio:sr.AudioData) -> None:
        data = audio.get_raw_data()
        data_queue.put(data)
        
    recorder.listen_in_background(source, record_callback, phrase_time_limit=record_timeout)
    
    print("Model loaded. Ready! \n")

    if flag:
        create_text("Generate a short simple salutation eager for the human to direct you")
    else:
        create_text("Generate one very short informal sentence to show that you are ready to listen")
    
    flag = False
    start_time = datetime.now()
    while (datetime.now() - start_time) < timedelta(seconds=3):
        try:
            if not data_queue.empty():
                phrase_complete = False
                now = datetime.now()
                
                if phrase_time and now - phrase_time > timedelta(seconds=phrase_timeout):
                    phrase_complete = True

                phrase_time = now

                # Combine audio data from queue.
                audio_data = b''.join(data_queue.queue)
                data_queue.queue.clear()

                # Process audio data.
                audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
                result = audio_model.transcribe(audio_np, fp16=torch.cuda.is_available())
                text = result['text'].strip()                        
                print(text)

            sleep(0.25)

        except KeyboardInterrupt:
            break
        
        selected_text = interpret_command_with_chatgpt(text)
        print("Selected Text: " + selected_text)
        text_publisher.publish(String(data=selected_text))
        process_voice_command(selected_text)

def create_text(prompting):
    response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": "You are a helpful assistant."},
                {"role": "user", "content": prompting}
            ]
        )
    response = response['choices'][0]['message']['content'].strip()
    play_sound(response)

def process_voice_command(msg):
    global text_subscriber
    text = str(msg).lower()
    digit_match = re.search(r'(-?[0-9]+)\b(?!\s*(degrees|d|Degrees))', text)
    degrees_match = re.search(r'([0-9]+)\s*(degrees|d|Degrees)?\b', text)

    if degrees_match:
        degrees = int(degrees_match.group(1))
        print(degrees)
    else:
        degrees = 45 
        
    if digit_match:
        distance_to_travel = int(digit_match.group(1))  
    else:
        distance_to_travel = 0
    
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
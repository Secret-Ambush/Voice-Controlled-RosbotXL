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
import argparse
import os
import numpy as np
import speech_recognition as sr
import whisper
import torch
from datetime import datetime, timedelta
from queue import Queue
from time import sleep
from sys import platform

load_dotenv()
api_key = os.getenv("API_KEY")
api_key2 = os.getenv("API_KEY_Eleven")
elevenlabs.set_api_key(api_key2)


class AudioPlayer:
    def __init__(self, voice="Bella"):
        self.voice = voice
        pygame.init()
        pygame.mixer.init()
    
    def play_sound(self, text):
        audio = generate(text, voice=self.voice)
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

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text_node')
        self.text_publisher = self.create_publisher(String, 'recognized_text', 10)
        self.flag = True
        self.speech_to_text_callback()
    
    def interpret_command_with_chatgpt(command):
        try:
            prompt_text = f"""Please convert {command} into a standardized navigation format by indicating the direction and distance. If no distance involved, 0cm.
            Your outputted direction should ONLY be limited to straight, left, right, or stop. 
            Your response should ONLY be (direction by distance).
            """

            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a helpful assistant."},
                    {"role": "user", "content": prompt_text}
                ]
            )

            return response['choices'][0]['message']['content'].strip()

        except Exception as e:
            print(f"An error occurred: {e}")
            return "Error processing command"
            
    def speech_to_text_callback(self):
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

        # Load / Download model
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
        
        if self.flag:
            self.createtext("Generate a short simple salutation eager for the human to direct you")
        else:
            self.createtext("Generate one very short informal sentence to show that you are ready to listen")
        
        
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
        selected_text = self.interpret_command_with_chatgpt(text)
        self.get_logger().info("Selected Text: " + selected_text)
        self.text_publisher.publish(String(data=selected_text))

    def createtext(self, prompting):
        completion = openai.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": "You are a bot."},
            {"role": "user", "content": prompting}
            ]
        )
        
        response = completion.choices[0].message.content
        AudioPlayer.play_sound(response)

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_command_processor')
        self.motion_command = Twist()
        self.text_subscriber = self.create_subscription(String, 'recognized_text', self.process_voice_command, 10)

    def process_voice_command(self, msg):
        text = msg.data
        digit_match = re.search(r'\b([1-9]|10|1[1-9]|20|30)\b', text)
        
        if digit_match:
            distance_to_travel = int(digit_match.group(0))
        else:
            distance_to_travel = 0

        self.get_logger().info("Linear Value: %d" % distance_to_travel)
        
        if "left" in text:
            self.get_logger().info("Command: Left")
            TurnRobot.turn_by_angle(45, 0.5)
            TurnRobot.move_linear(distance_to_travel, 0.2)  
            
        if "right" in text:
            self.get_logger().info("Command: Right")
            TurnRobot.turn_by_angle(-45, 0.5)
            TurnRobot.move_linear(distance_to_travel, 0.2)
        
        elif "straight" in text:
            self.get_logger().info("Command: Right")
            SpeechToTextNode.createtext("Generate a simple sentence to say that you are moving straight")
            TurnRobot.move_linear(distance_to_travel, 0.2)
        
        elif "stop" in text:
            SpeechToTextNode.createtext("Generate a simple sentence to say that you stopped")
            self.get_logger().info("Command: Stop") 
            VoiceCommandProcessor.stop_robot()

    def stop_robot(self):
        self.get_logger().info("Stopping robot")
        self.motion_command.linear.x = 0.0
        self.motion_command.angular.z = 0.0
        self.motion_publisher.publish(self.motion_command)

class TurnRobot(Node):
    def __init__(self):
        super().__init__('turn_robot')
        self.motion_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def turn_by_angle(self, angle_degrees, angular_velocity):
        angle_radians = math.radians(angle_degrees)
        
        motion_command = Twist()
        motion_command.angular.z = angular_velocity if angle_degrees > 0 else -angular_velocity
        
        turn_duration_sec = abs(angle_radians / angular_velocity)
        
        self.get_logger().info(f"Turning {'left' if angle_degrees > 0 else 'right'} by {abs(angle_degrees)} degrees")
        
        end_time = self.get_clock().now() + Duration(seconds=turn_duration_sec)
        while self.get_clock().now() < end_time:
            self.motion_publisher.publish(motion_command)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        motion_command.angular.z = 0.0
        self.motion_publisher.publish(motion_command)
        self.get_logger().info("Turn complete.")

    def move_linear(self, distance_cm, linear_velocity):
        # Convert distance from cm to meters
        distance_m = distance_cm / 100.0
        
        motion_command = Twist()
        motion_command.linear.x = linear_velocity
        
        move_duration_sec = abs(distance_m / linear_velocity)
        
        self.get_logger().info(f"Moving forward {distance_cm} cm")
        
        end_time = self.get_clock().now() + Duration(seconds=move_duration_sec)
        while self.get_clock().now() < end_time:
            self.motion_publisher.publish(motion_command)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        motion_command.linear.x = 0.0
        self.motion_publisher.publish(motion_command)
        self.get_logger().info("Movement complete.")
       
def main(args=None):
    
    rclpy.init(args=args)
    node = SpeechToTextNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

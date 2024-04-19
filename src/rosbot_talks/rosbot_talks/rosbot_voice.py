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
        if self.flag:
            self.createtext("Generate a short simple salutation eager for the human to direct you")
        else:
            self.createtext("Generate one very short informal sentence to show that you are ready to listen")
        
        self.flag = False
        recognizer = sr.Recognizer()
        
        try:
            with sr.Microphone() as source:
                self.get_logger().info("Listening now: ")
                audio = recognizer.listen(source, timeout=3)
                self.get_logger().info("Stopped Listening")
                self.createtext("Generate a very short informal sentence to say that you got the command")
                text = recognizer.recognize_google(audio, show_all=True)
                self.get_logger().info(str(text))
            
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

            selected_text = self.interpret_command_with_chatgpt(selected_text)
            self.get_logger().info("Selected Text: " + selected_text)
            self.text_publisher.publish(String(data=selected_text))
        
        except Exception as e:
            self.get_logger().error('Failed to recognize speech: ' + str(e))

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

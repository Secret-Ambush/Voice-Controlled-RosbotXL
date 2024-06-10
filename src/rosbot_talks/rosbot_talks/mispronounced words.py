import openai
import os
from dotenv import load_dotenv
import re

load_dotenv()

api_key = os.getenv("API_KEY")
if not api_key:
    raise ValueError("API_KEY environment variable not set.")

openai.api_key = api_key

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

sample_command = "Can you move turn left 45 degrees and then move backwards 10 units?"
print(sample_command)
final = interpret_command_with_chatgpt(sample_command)
print(final)

text = str(final).lower()
digit_match = re.search(r'\b(-?[0-9]+)\b(?!\s*(degrees|d|Degrees))', text)
degrees_match = re.search(r'\b([0-9]+)\s*(degrees|d|Degrees)?\b', text)

if degrees_match:
    degrees = int(degrees_match.group(1))
else:
    degrees = 45 
    
if digit_match:
        distance_to_travel = int(digit_match.group(1))
else:
    distance_to_travel = 0
    
print(f"Degrees: {degrees}")
print(f"Distance: {distance_to_travel}")

import json
from llamaapi import LlamaAPI
import re

# Initialize the SDK
llama = LlamaAPI("LA-18ef5c088a3a42e388278a5de3d984cc8e92d4ef24a2408eaed82da6b9126d14")

command = "Can you move turn left 45 degrees and then move backwards 10 units?"
prompt_text = f"""Perform the following operations on the provided {command} given by a human that involves direction and distance:
            You should determine the direction as straight, left, right, or stop ONLY.
            Output the corrected command ONLY. No flavour text.
            If the command is "turn around" or any synonym of it, the output should be "turn left 360 degrees." 
            However, if the command specifies a direction such as clockwise or anti-clockwise, the output should be "turn right 360 degrees" and "turn left 360 degrees" respectively.
            If the direction is like backwards, specify the direction as 'straight' and convert the distance to negative.
            In command mentions anything like diagonal movement, the response should be 'turn left 15 degrees and move straight specified units'.
            Please make sure to rectify any potential spelling errors or homophone mistakes.
        """
        
# Build the API request
api_request_json = {
    "model": "llama3-8b",
    "messages": [
        {"role": "user", "content": prompt_text},
    ]
}

# Execute the Request
response = llama.run(api_request_json)
final = json.dumps(response.json(), indent=2)

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

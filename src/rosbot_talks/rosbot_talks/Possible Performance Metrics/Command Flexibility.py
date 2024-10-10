import openai

predefined_commands = [
    "move forward",
    "turn left",
    "turn right",
    "stop",
    "move backward",
    "spin around"
]

def gpt4o_match_command(user_command):
    prompt = f"The following are possible commands for a robot: {', '.join(predefined_commands)}. Which command is closest in meaning to: '{user_command}'?"
    
    response = openai.ChatCompletion.create(
        model="gpt-4o",
        messages=[
            {"role": "system", "content": "You are an assistant that helps match user commands to predefined robot commands."},
            {"role": "user", "content": prompt}
        ]
    )
    
    best_match = response.choices[0].message['content'].strip()
    return best_match

user_commands = [
    "please move forward",
    "advance",
    "can you turn to the left",
    "halt",
    "go backwards",
    "rotate around"
]

for command in user_commands:
    matched_command = gpt4o_match_command(command)
    print(f"User Command: '{command}' -> Matched Command: '{matched_command}'")
import re

sample_command = "go straight -10 units then turn left 45 degrees"
text = sample_command.lower()

digit_match = re.search(r'(-?[0-9]+)\b(?!\s*(degrees|d|Degrees))', text)
degrees_match = re.search(r'([0-9]+)[ \t]*(degrees|d|Degrees)\b', text)

# degrees = int(degrees_match.group(1)) if degrees_match else None
distance_to_travel = (int(digit_match.group(1)) if digit_match else None)
degrees = (int(degrees_match.group(1)) if degrees_match else None)

print(distance_to_travel)
print(degrees)

words = text.split()
for word in words:
    if "left" in word:
        print("Command: Left")
        print(f"Degrees: {degrees}")
        print("Turning by angle")

    if "right" in word:
        print("Command: Right")
        print("Turning by angle")

    if "straight" in word:
        if distance_to_travel < 0:
            print("backwards")
        else:
            print("forward")
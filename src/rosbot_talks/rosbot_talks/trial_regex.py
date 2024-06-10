import re

sample_command = "turn left 45 degrees and then move straight -10 units"
text = sample_command.lower()

digit_match = re.search(r'\b(-?[0-9]+)\b(?!\s*(degrees|d|Degrees))', text)
# degrees_match = re.search(r'\b([0-9]+)\s*(degrees|d|Degrees)?\b', text)
print(digit_match)
# degrees = int(degrees_match.group(1)) if degrees_match else None
distance = int(digit_match.group(1)) if digit_match else None

# print("Degrees:", degrees)
print("Distance:", distance)

import jiwer

# Ground truth (reference) transcriptions and the system's predicted transcriptions
reference_texts = [
    "move forward",
    "turn left",
    "stop the robot",
    "move backwards",
    "turn right and stop"
]

predicted_texts = [
    "move forward",
    "turn left",
    "stop robot",
    "move backward",
    "turn right stop"
]

# Calculate Word Error Rate (WER)
wer = jiwer.wer(reference_texts, predicted_texts)

print(f"Word Error Rate (WER): {wer:.2f}")

# Additional metrics (optional)
# You can also calculate other metrics such as Mer (Match Error Rate), Wil (Word Information Lost)
mer = jiwer.mer(reference_texts, predicted_texts)
wil = jiwer.wil(reference_texts, predicted_texts)

print(f"Match Error Rate (MER): {mer:.2f}")
print(f"Word Information Lost (WIL): {wil:.2f}")
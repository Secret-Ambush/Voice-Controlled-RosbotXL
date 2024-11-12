from transformers import pipeline
import torch

model_id = "meta-llama/Llama-3.2-3B-Instruct"
pipe = pipeline(
    "text-generation",
    model=model_id,
    torch_dtype=torch.bfloat16,
)

messages = [{"role": "user", "content": "What are quantum computers?"}]
outputs = pipe(messages, max_new_tokens=150)
response = outputs[0]["generated_text"]
print(response)
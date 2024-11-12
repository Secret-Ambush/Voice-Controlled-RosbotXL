from transformers import AutoTokenizer, AutoModelForCausalLM
import torch

tokenizer = AutoTokenizer.from_pretrained("meta-llama/Llama-3.2-1B")
model = AutoModelForCausalLM.from_pretrained("meta-llama/Llama-3.2-1B")
input_text = "The key to life is"
inputs = tokenizer(input_text, return_tensors="pt").to("cuda")  # Move inputs to GPU if available

output = model.generate(**inputs, max_new_tokens=50)
generated_text = tokenizer.decode(output[0], skip_special_tokens=True)

print(generated_text)
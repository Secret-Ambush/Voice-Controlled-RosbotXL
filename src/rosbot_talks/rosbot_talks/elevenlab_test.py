from elevenlabs import play, save, stream, Voice, VoiceSettings, generate
import elevenlabs
from dotenv import load_dotenv
import os

load_dotenv()
api_key = os.getenv("API_KEY_Eleven")

def text_stream():
   yield "Hi! I'm Brian "
   yield "I'm an artificial voice made by ElevenLabs "

elevenlabs.set_api_key(api_key)
audio = generate(
   text="Hello there!",
   voice="Brian",
   model='eleven_turbo_v2_5'
)


play(audio)
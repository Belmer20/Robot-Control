from pydantic import BaseModel



class voice_signal(BaseModel):
    audio: bytes

class video_frames(BaseModel):
    ... 
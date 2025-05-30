from pydantic import BaseModel

class Pose(BaseModel):
    x: float; y: float; z: float = 0.0
    q1: float; q2: float; q3: float; q4: float

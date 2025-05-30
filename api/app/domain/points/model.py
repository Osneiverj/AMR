from beanie import Document
from datetime import datetime
from app.core.schemas import Pose

class Point(Document):
    name: str
    type: str                        # way | dock | station
    map_id: str                      # _id (str) del Map
    target: Pose
    face_target: Pose | None = None
    created: datetime = datetime.utcnow()

    class Settings:
        name = "points"

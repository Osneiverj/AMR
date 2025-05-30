from beanie import Document
from datetime import datetime
from typing import List

class Mission(Document):
    name: str
    map_id: str                      # referencia por id
    sequence: List[str]              # lista de point._id
    loop: bool = False
    created: datetime = datetime.utcnow()

    class Settings:
        name = "missions"

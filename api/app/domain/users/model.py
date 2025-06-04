from datetime import datetime
from beanie import Document
from pydantic import Field

class User(Document):
    username: str = Field(unique=True)
    hashed_password: str
    role: str = "user"
    created: datetime = Field(default_factory=datetime.utcnow)

    class Settings:
        name = "users"

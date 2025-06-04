from pydantic import BaseModel

class UserCreate(BaseModel):
    username: str
    password: str
    role: str = "user"

class UserOut(BaseModel):
    id: str
    username: str
    role: str


class PasswordChange(BaseModel):
    new_password: str

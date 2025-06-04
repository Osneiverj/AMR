from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordRequestForm


from .schemas import UserCreate, UserOut, PasswordChange

from .service import (
    authenticate_user,
    create_token,
    create_user,
    get_current_active_user,
    require_admin,
    update_password,

)

router = APIRouter(prefix="/auth", tags=["auth"])


@router.post("/register", response_model=UserOut)
async def register(
    user_in: UserCreate,
    current_user=Depends(require_admin),
):

    user = await create_user(user_in.username, user_in.password, user_in.role)
    return UserOut(id=str(user.id), username=user.username, role=user.role)


@router.post("/login")
async def login(form_data: OAuth2PasswordRequestForm = Depends()):
    user = await authenticate_user(form_data.username, form_data.password)
    if not user:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Incorrect username or password")
    token = create_token({"sub": user.username})
    return {"access_token": token, "token_type": "bearer"}


@router.get("/me", response_model=UserOut)
async def read_users_me(current_user=Depends(get_current_active_user)):
    return UserOut(id=str(current_user.id), username=current_user.username, role=current_user.role)


@router.post("/change-password")
async def change_password(
    data: PasswordChange,
    current_user=Depends(get_current_active_user),
):
    await update_password(current_user, data.new_password)
    return {"status": "ok"}


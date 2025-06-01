# api/app/main.py
import logging
from fastapi import FastAPI, Request, status
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware

# Importa tus routers explícitamente
from app.domain.maps.router import router as maps_router
from app.domain.missions.router import router as missions_router
from app.domain.points.router import router as points_router
# Añade aquí cualquier otro router que crees en el futuro
# from app.domain.otro_modulo.router import router as otro_router

app = FastAPI(title="AGV API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=[*settings.cors_origins] if isinstance(settings.cors_origins, list)
                  else [settings.cors_origins],
    allow_credentials=True, # Considera si necesitas cookies/auth headers
    allow_methods=["*"],
    allow_headers=["*"],
)

# ── INCLUYE LOS ROUTERS EXPLÍCITAMENTE ─────────────────────────
app.include_router(maps_router)
app.include_router(missions_router)
app.include_router(points_router)
# app.include_router(otro_router)
# ───────────────────────────────────────────────────────────────────

@app.on_event("startup")
async def startup_event(): # Renombrado para evitar conflicto con una posible variable 'startup'
    await init_db()
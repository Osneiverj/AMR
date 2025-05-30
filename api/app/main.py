# app/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from importlib import import_module
from pathlib import Path
from app.core.config import settings
from app.core.db import init_db

app = FastAPI(title="AGV API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=[*settings.cors_origins] if isinstance(settings.cors_origins, list)
                  else [settings.cors_origins],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ── CARGA LOS ROUTERS EN ORDEN ALFABÉTICO ─────────────────────────
router_paths = sorted(
    Path(__file__).parent.joinpath("domain").rglob("router.py"),
    key=lambda p: p.parent.name          # maps → missions → points …
)
for p in router_paths:
    mod = import_module(f"app.domain.{p.parent.name}.router")
    app.include_router(mod.router)
# ───────────────────────────────────────────────────────────────────

@app.on_event("startup")
async def startup():
    await init_db()

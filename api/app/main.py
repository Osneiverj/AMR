# api/app/main.py
import logging
from fastapi import FastAPI, Request, status
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware

from datetime import datetime

from app.core.config import settings
from app.core.db import init_db
from app.core.logging_config import setup_logging_config

# Si decidiste usar la carga explícita de routers (Paso 4)
from app.domain.maps.router import router as maps_router
from app.domain.missions.router import router as missions_router
from app.domain.points.router import router as points_router
from app.domain.users.router import router as auth_router
from app.domain.users.service import ensure_default_admin

# Configura el logging ANTES de que cualquier otra cosa suceda,
# especialmente antes de crear la instancia de FastAPI.
setup_logging_config() 

# Obtén un logger para este módulo después de que la configuración se haya aplicado.
logger = logging.getLogger(__name__) 

app = FastAPI(
    title="AGV API",
    version="0.1.0", # Es bueno versionar tu API
    # openapi_url=f"{settings.API_V1_STR}/openapi.json" # Si tienes un prefijo de API
)

# Middleware de CORS (como lo tenías)
app.add_middleware(
    CORSMiddleware,
    allow_origins=[str(origin) for origin in settings.cors_origins] if isinstance(settings.cors_origins, list) else [settings.cors_origins],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Routers (explícitos como en Paso 4)
app.include_router(auth_router)
app.include_router(maps_router)
app.include_router(missions_router)
app.include_router(points_router)

# Handler de excepciones global para errores no controlados (500)
@app.exception_handler(Exception)
async def generic_exception_handler(request: Request, exc: Exception):
    logger.error(
        f"Error no controlado en la solicitud: {request.method} {request.url}",
        exc_info=exc, # Pasa el objeto de excepción para el traceback
        extra={"props": {"type": "unhandled_exception"}}
    )
    return JSONResponse(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        content={"detail": "Ocurrió un error interno en el servidor."},
    )

# Handler para errores de validación de Pydantic/FastAPI (422)
# Esto es opcional si quieres customizar el formato del error 422,
# FastAPI ya provee uno bueno por defecto.
# @app.exception_handler(RequestValidationError)
# async def validation_exception_handler(request: Request, exc: RequestValidationError):
#     logger.warning(
#         f"Error de validación: {request.method} {request.url}",
#         extra={"props": {"errors": exc.errors(), "body": exc.body, "type": "validation_error"}}
#     )
#     return JSONResponse(
#         status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
#         content={"detail": exc.errors(), "body": exc.body}, # O un formato más simple
#     )

@app.on_event("startup")
async def startup_event():
    logger.info(
        "Iniciando aplicación FastAPI...",
        extra={"props": {"app_name": app.title, "app_version": app.version}}
    )
    await init_db()
    await ensure_default_admin(settings.default_admin_username, settings.default_admin_password)
    logger.info("Base de datos inicializada.")
    logger.info("Aplicación FastAPI iniciada correctamente.")

@app.on_event("shutdown")
async def shutdown_event():
    logger.info("Aplicación FastAPI apagándose...")
    # Lógica de limpieza si es necesaria
    logger.info("Aplicación FastAPI apagada correctamente.")

# Endpoint de Health Check
@app.get("/health", tags=["Health"], status_code=status.HTTP_200_OK)
async def health_check():
    # Usar 'extra' para añadir campos estructurados
    logger.info(
        "Health check solicitado.", 
        extra={"props": {"component": "API", "status_requested": "healthy"}}
    )
    return {"status": "healthy", "timestamp": datetime.utcnow().isoformat()}
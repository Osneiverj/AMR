# api/Dockerfile

# 1. Usar una imagen base oficial de Python específica y delgada
FROM python:3.11-slim

# 2. Establecer variables de entorno para Python (opcional pero recomendado)
ENV PYTHONDONTWRITEBYTECODE 1  # Evita que Python escriba archivos .pyc
ENV PYTHONUNBUFFERED 1         # Fuerza que los logs de Python salgan directamente, útil para Docker

# 3. Establecer el directorio de trabajo en el contenedor
WORKDIR /app

# 4. Crear un grupo y un usuario no-root para ejecutar la aplicación
#    (Mejora la seguridad. UID/GID 1000 es común para el primer usuario no-root)
RUN groupadd -r appgroup --gid 1000 && \
    useradd -r -g appgroup --uid 1000 --no-create-home appuser
    # -r crea un usuario del sistema
    # --no-create-home porque no necesitamos un directorio home para esta app simple
    # WORKDIR /app ya es el directorio de la app

# 5. Copiar el archivo de dependencias PRIMERO
COPY --chown=appuser:appgroup requirements.txt .

# 6. Instalar dependencias del sistema necesarias para ciertos paquetes de Python
RUN apt-get update && \
    apt-get install -y --no-install-recommends graphviz graphviz-dev && \
    rm -rf /var/lib/apt/lists/*

# 7. Instalar las dependencias de Python
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# 8. Copiar el resto del código de la aplicación al directorio de trabajo
#    Usa --chown para que los archivos sean propiedad del usuario no-root directamente
COPY --chown=appuser:appgroup app ./app

# 9. Cambiar al usuario no-root
USER appuser

# 10. Exponer el puerto en el que corre la aplicación DENTRO del contenedor
EXPOSE 8000

# 11. Comando para ejecutar la aplicación
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000", "--workers", "1"]
# Añadido --workers 1 como un punto de partida razonable para Uvicorn
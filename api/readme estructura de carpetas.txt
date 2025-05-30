tu-proyecto/
│
├─ docker-compose.yml            <-- el que ya tienes (simulador, UI…)
├─ docker-compose.api.yml        <-- SOLO declara Mongo y FastAPI
│                                   (extiende al anterior)
└─ api/                          <-- código del backend
   ├─ Dockerfile                 <-- construye la imagen FastAPI
   ├─ requirements.txt           <-- librerías Python
   └─ app/                       <-- paquete de tu aplicación
      ├─ main.py                 <-- crea FastAPI y auto-carga módulos
      ├─ core/                   <-- utilidades comunes
      │   ├─ config.py           <-- lee variables de entorno (Mongo URI, etc.)
      │   └─ db.py               <-- conecta a Mongo con Beanie
      └─ domain/                 <-- aquí metes **un sub-directorio por módulo**
          ├─ maps/
          │   ├─ model.py        <-- definición del documento Mongo
          │   ├─ service.py      <-- lógica (guardar archivos, etc.)
          │   └─ router.py       <-- rutas REST (/maps …)
          ├─ points/
          └─ missions/

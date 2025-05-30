from beanie import Document
from datetime import datetime

class Map(Document):
    name: str         # único
    pgm: str          # nombre del .pgm en /maps
    yaml: str         # nombre del .yaml
    created: datetime

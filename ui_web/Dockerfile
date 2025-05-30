# ---------- build stage ----------
FROM node:18-alpine AS build
WORKDIR /app

# Instalar npm (16 MB) – sólo en la capa de build
RUN apk add --no-cache npm

COPY package.json vite.config.js index.html ./
COPY src ./src
COPY public ./public
COPY postcss.config.js tailwind.config.js ./

RUN npm install --no-fund 

RUN npm run build

#---------- production stage ----------
FROM nginx:1.25-alpine
COPY --from=build /app/dist /usr/share/nginx/html
COPY docker/nginx.conf /etc/nginx/conf.d/default.conf
EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]
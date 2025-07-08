// src/components/Map.jsx
import { MapContainer, useMap } from 'react-leaflet';
import { useEffect, useRef, useState } from 'react';
import L from 'leaflet';
import ROSLIB from 'roslib';
import ros from '../services/rosService';
import { FOOTPRINT } from '../utils/constants';
import { useData } from '../context/DataContext';
import { PointsAPI } from '../services/api';
import 'leaflet/dist/leaflet.css';

/* CRS.Simple = plano en metros.
  Convención:  (x, y) ROS  ↔  (lng, lat) Leaflet.                 */

export default function Map() {
  /* ───────── refs & state ─────────────────────────────────────── */
  const mapRef      = useRef(null);
  const gridLayer   = useRef(null);
  const robotLayer  = useRef(null);
  const scanLayer   = useRef(L.layerGroup());   // nube de puntos
  const canvas      = useRef(document.createElement('canvas'));
  const poseRef     = useRef({ x: 0, y: 0, yaw: 0 });
  const initPubRef  = useRef(null);
  const goalPubRef  = useRef(null);

  const [pose, setPose] = useState({ x: 0, y: 0, yaw: 0 });
  const [gridInfo, setGridInfo] = useState(null); // info de /map
  const scanSubRef = useRef(null); // <-- NUEVO
  const [mode, setMode] = useState(null); // initial | goal | point
  const { selectedMap, setPoints } = useData();

  // Publishers for initial pose and goal pose
  useEffect(() => {
    initPubRef.current = ros.advertise(
      '/initialpose',
      'geometry_msgs/PoseWithCovarianceStamped'
    );
    goalPubRef.current = ros.advertise(
      '/goal_pose',
      'geometry_msgs/PoseStamped'
    );
    // Explicitly advertise
    initPubRef.current.advertise();
    goalPubRef.current.advertise();
    return () => {
      initPubRef.current.unadvertise();
      goalPubRef.current.unadvertise();
    };
  }, []);

  /* ───────── 1.  /tf  →  pose ─────────────────────────────────── */
  useEffect(() => {
   const sub = ros.subscribe('/tf', 'tf2_msgs/TFMessage', msg => {
    const t = msg.transforms.find(tr => tr.child_frame_id === 'base_footprint');
    if (!t) return;
    const { x, y, z, w } = t.transform.rotation;
    const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    const newPose = {
      x: t.transform.translation.x,
      y: t.transform.translation.y,
      yaw
    };
    poseRef.current = newPose;
    setPose(newPose);
   });
   return () => sub.unsubscribe();
  }, []);

  /* ───────── 2.  /map  →  occupancy overlay ───────────────────── */
  useEffect(() => {
   const ctx = canvas.current.getContext('2d');
   ctx.imageSmoothingEnabled = false;

   const sub = ros.subscribe('/map', 'nav_msgs/OccupancyGrid', msg => {
    setGridInfo(msg.info);                             // guarda metadata
    const { width, height } = msg.info;

    if (canvas.current.width !== width)  canvas.current.width  = width;
    if (canvas.current.height !== height) canvas.current.height = height;

    const img = ctx.createImageData(width, height);
    msg.data.forEach((v, i) => {
      const c = v < 0 ? 200 : v === 0 ? 255 : 0;      // gris / blanco / negro
      img.data.set([c, c, c, 255], i * 4);
    });
    ctx.putImageData(img, 0, 0);

    const bounds = gridBounds(msg.info);
    if (gridLayer.current) {
      gridLayer.current.setUrl(canvas.current.toDataURL());
      gridLayer.current.setBounds(bounds);
    } else {
      gridLayer.current = L.imageOverlay(canvas.current.toDataURL(), bounds, {
       opacity: 0.75
      }).addTo(mapRef.current);
      /* confina pan + ajusta zoom máximo */
      mapRef.current.fitBounds(bounds);
      mapRef.current.setMaxBounds(bounds);
      mapRef.current.options.maxBoundsViscosity = 1.0;
      mapRef.current.setMaxZoom(mapRef.current.getZoom() + 6);
    }
   });
   return () => sub.unsubscribe();
  }, []);

  /* ───────── 3.  footprint sobre la pose ──────────────────────── */
  useEffect(() => {
   if (!mapRef.current || !gridInfo) return;

   if (robotLayer.current) robotLayer.current.remove();
   const { x, y, yaw } = pose;
   const rot = (vx, vy) => [
    y + vy * Math.cos(yaw) + vx * Math.sin(yaw), // lat  (= Y)
    x + vx * Math.cos(yaw) - vy * Math.sin(yaw)  // lng  (= X)
   ];
   const verts = FOOTPRINT.map(([vx, vy]) => rot(vx, vy));

   robotLayer.current = L.polygon(verts, {
    color: '#1d4ed8',
    weight: 2,
    fillOpacity: 0.4
   }).addTo(mapRef.current);
  }, [pose, gridInfo]);

  /* ───────── 4.  nube de puntos LIDAR ─────────────────────────── */
  // Suscripción LIDAR: solo una vez cuando gridInfo esté listo
  useEffect(() => {
    if (!scanLayer.current || !mapRef.current || !gridInfo) return;
    if (scanSubRef.current) return; // ya suscrito

    scanLayer.current.addTo(mapRef.current);

    scanSubRef.current = ros.subscribe(
      '/scan',
      'sensor_msgs/LaserScan',
      msg => {
        scanLayer.current.clearLayers();
        const { angle_min, angle_increment, ranges } = msg;
        // Usamos poseRef.current, que siempre tiene la última pose
        const { x, y, yaw } = poseRef.current;
        ranges.forEach((r, i) => {
          if (!isFinite(r)) return;
          const angle = angle_min + i * angle_increment + yaw;
          const lx = x + r * Math.cos(angle);
          const ly = y + r * Math.sin(angle);
          L.circleMarker([ly, lx], { radius: 1 }).addTo(scanLayer.current);
        });
      }
    );
    // No return aquí: la desuscripción se hace solo al desmontar
    // para evitar re-suscribir/desuscribir en cada cambio de gridInfo
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [gridInfo, mapRef.current, scanLayer.current]);

  // Desuscribir solo al desmontar el componente
  useEffect(() => {
    return () => {
      if (scanSubRef.current) {
        scanSubRef.current.unsubscribe();
        scanSubRef.current = null;
      }
    };
  }, []);

  /* ───────── helper: bounds en CRS.Simple ─────────────────────── */
  const gridBounds = info => {
   const { width, height, resolution, origin } = info;
   return [
    [origin.position.y, origin.position.x],
    [origin.position.y + height * resolution,
     origin.position.x + width  * resolution]
   ];
  };

  const quatFromYaw = yaw => ({
    x: 0,
    y: 0,
    z: Math.sin(yaw / 2),
    w: Math.cos(yaw / 2)
  });

  /* ───────── inicia mapa ───────────────────────────────────────── */
  function MapInit() {
   const map = useMap();
   mapRef.current = map;
   return null;
  }

  async function onMapClick(e) {
    const { lat, lng } = e.latlng;
    if (mode === 'initial' && initPubRef.current) {
      const yaw = parseFloat(prompt('Yaw (grados)', '0')) * Math.PI / 180 || 0;
      const msg = new ROSLIB.Message({
        header: { frame_id: 'map' },
        pose: {
          pose: {
            position: { x: lng, y: lat, z: 0 },
            orientation: quatFromYaw(yaw)
          },
          covariance: Array(36).fill(0)
        }
      });
      initPubRef.current.publish(msg);
      setMode(null);
    } else if (mode === 'goal' && goalPubRef.current) {
      const yaw = parseFloat(prompt('Yaw (grados)', '0')) * Math.PI / 180 || 0;
      const msg = new ROSLIB.Message({
        header: { frame_id: 'map' },
        pose: {
          position: { x: lng, y: lat, z: 0 },
          orientation: quatFromYaw(yaw)
        }
      });
      goalPubRef.current.publish(msg);
      setMode(null);
    } else if (mode === 'point' && selectedMap) {
      const name = prompt('Nombre del punto');
      if (!name) return;
      const yaw = parseFloat(prompt('Yaw (grados)', '0')) * Math.PI / 180 || 0;
      await PointsAPI.create(
        {
          name,
          type: 'way',
          map_id: selectedMap,
          target: {
            x: lng,
            y: lat,
            z: 0,
            q1: 0,
            q2: 0,
            q3: quatFromYaw(yaw).z,
            q4: quatFromYaw(yaw).w
          }
        }
      );
      setPoints(await PointsAPI.list(selectedMap));
      setMode(null);
    }
  }

  return (
    <div className="relative">
      <MapContainer
        crs={L.CRS.Simple}
        center={[0, 0]}
        zoom={0}
        minZoom={-4}
        style={{ height: '26rem', width: '100%' }}
        scrollWheelZoom
        onClick={onMapClick}
      >
        <MapInit />
      </MapContainer>
      <div className="absolute top-2 left-2 z-10 flex gap-2">
        <button
          className={`btn ${mode === 'initial' ? 'bg-blue-500 text-white' : 'bg-white'}`}
          onClick={() => setMode(mode === 'initial' ? null : 'initial')}
        >
          Pose inicial
        </button>
        <button
          className={`btn ${mode === 'goal' ? 'bg-blue-500 text-white' : 'bg-white'}`}
          onClick={() => setMode(mode === 'goal' ? null : 'goal')}
        >
          Objetivo
        </button>
        <button
          className={`btn ${mode === 'point' ? 'bg-blue-500 text-white' : 'bg-white'}`}
          onClick={() => setMode(mode === 'point' ? null : 'point')}
        >
          Nuevo punto
        </button>
      </div>
    </div>
  );}


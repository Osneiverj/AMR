// src/components/Map.jsx
import { MapContainer, useMap } from 'react-leaflet';
import { useEffect, useRef, useState } from 'react';
import L from 'leaflet';
import ros from '../services/rosService';
import { FOOTPRINT } from '../utils/constants';
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

  const [pose, setPose] = useState({ x: 0, y: 0, yaw: 0 });
  const [gridInfo, setGridInfo] = useState(null); // info de /map

  /* ───────── 1.  /tf  →  pose ─────────────────────────────────── */
  useEffect(() => {
    const sub = ros.subscribe('/tf', 'tf2_msgs/TFMessage', msg => {
      const t = msg.transforms.find(tr => tr.child_frame_id === 'base_footprint');
      if (!t) return;
      const { x, y, z, w } = t.transform.rotation;
      const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
      setPose({
        x: t.transform.translation.x,
        y: t.transform.translation.y,
        yaw
      });
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
  useEffect(() => {
    if (!mapRef.current || !gridInfo) return;
    scanLayer.current.addTo(mapRef.current);

    const sub = ros.subscribe('/scan', 'sensor_msgs/LaserScan', msg => {
      scanLayer.current.clearLayers();
      const { angle_min, angle_increment, ranges } = msg;

      ranges.forEach((r, i) => {
        if (!isFinite(r)) return;
        const angle = angle_min + i * angle_increment + pose.yaw;
        const lx = pose.x + r * Math.cos(angle);
        const ly = pose.y + r * Math.sin(angle);
        L.circleMarker([ly, lx], { radius: 1, color: '#ff3333' })
          .addTo(scanLayer.current);
      });
    });
    return () => sub.unsubscribe();
  }, [pose, gridInfo]);

  /* ───────── helper: bounds en CRS.Simple ─────────────────────── */
  const gridBounds = info => {
    const { width, height, resolution, origin } = info;
    return [
      [origin.position.y, origin.position.x],
      [origin.position.y + height * resolution,
       origin.position.x + width  * resolution]
    ];
  };

  /* ───────── inicia mapa ───────────────────────────────────────── */
  function MapInit() {
    const map = useMap();
    mapRef.current = map;
    return null;
  }

  return (
    <MapContainer
      crs={L.CRS.Simple}
      center={[0, 0]}
      zoom={0}
      minZoom={-4}
      style={{ height: '26rem', width: '100%' }}
      scrollWheelZoom
    >
      <MapInit />
    </MapContainer>
  );
}

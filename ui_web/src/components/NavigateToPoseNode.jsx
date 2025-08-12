import React, { memo, useEffect, useState } from 'react';
import { Handle, Position } from 'reactflow';
import { get as apiGet } from '../services/apiService';

const NavigateToPoseNode = ({ data, id }) => {
  const [points, setPoints] = useState([]);

  useEffect(() => {
    console.log(`NavigateToPoseNode (${id}): mapId recibido:`, data.mapId);
    const fetchPoints = async () => {
      if (data.mapId) {
        try {
          console.log(`NavigateToPoseNode (${id}): Realizando fetch de puntos para el mapa...`);
          const pointsData = await apiGet(`/points/?map_id=${data.mapId}`);
          console.log(`NavigateToPoseNode (${id}): Puntos recibidos:`, pointsData);
          setPoints(pointsData);
        } catch (error) {
          console.error(`NavigateToPoseNode (${id}): Error al obtener los puntos:`, error);
        }
      }
    };
    fetchPoints();
  }, [data.mapId, id]);

  const onChange = (evt) => {
    if (data.onChange) {
      data.onChange(id, { pointId: evt.target.value });
    }
  };

  return (
    <div style={{ background: '#D1E7DD', padding: '10px', borderRadius: '5px', border: '1px solid #28a745', width: 180 }}>
      <Handle type="target" position={Position.Top} />
      <div>
        <strong>Navigate to Pose</strong>
      </div>
      <div style={{ marginTop: '5px' }}>
        <label style={{ fontSize: '12px' }}>Point:</label>
        <select value={data.pointId || ''} onChange={onChange} style={{ width: '100%', padding: '2px', fontSize: '12px' }}>
          <option value="">Select a point</option>
          {points.map(point => (
            <option key={point.id} value={point.id}>
              {point.name}
            </option>
          ))}
        </select>
      </div>
      <Handle type="source" position={Position.Bottom} />
    </div>
  );
};

export default memo(NavigateToPoseNode);

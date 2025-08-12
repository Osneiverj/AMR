import React, { memo } from 'react';
import { Handle, Position } from 'reactflow';

const NavigateToPoseNode = ({ data }) => {
  return (
    <div style={{ background: '#D1E7DD', padding: '10px', borderRadius: '5px', border: '1px solid #28a745', width: 180 }}>
      <Handle type="target" position={Position.Top} />
      <div>
        <strong>Navigate to Pose</strong>
      </div>
      <div style={{ marginTop: '5px' }}>
        <label style={{ fontSize: '12px' }}>Point ID:</label>
        <input type="text" defaultValue={data.pointId || ''} style={{ width: '100%', padding: '2px', fontSize: '12px' }} />
      </div>
      <Handle type="source" position={Position.Bottom} />
    </div>
  );
};

export default memo(NavigateToPoseNode);

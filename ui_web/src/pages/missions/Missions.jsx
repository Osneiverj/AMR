import React, { useState, useCallback } from 'react';
import ReactFlow, { addEdge, applyEdgeChanges, applyNodeChanges, Background, Controls, MiniMap } from 'reactflow';
import 'reactflow/dist/style.css';

import './MissionEditor.css';

const initialNodes = [
  {
    id: '1',
    type: 'input',
    data: { label: 'Start Mission' },
    position: { x: 250, y: 5 },
  },
];

const MissionEditor = () => {
  const [nodes, setNodes] = useState(initialNodes);
  const [edges, setEdges] = useState([]);

  const onNodesChange = useCallback(
    (changes) => setNodes((nds) => applyNodeChanges(changes, nds)),
    [setNodes]
  );
  const onEdgesChange = useCallback(
    (changes) => setEdges((eds) => applyEdgeChanges(changes, eds)),
    [setEdges]
  );
  const onConnect = useCallback(
    (connection) => setEdges((eds) => addEdge(connection, eds)),
    [setEdges]
  );

  return (
    <div style={{ height: 'calc(100vh - 64px)' }}>
      <ReactFlow
        nodes={nodes}
        edges={edges}
        onNodesChange={onNodesChange}
        onEdgesChange={onEdgesChange}
        onConnect={onConnect}
        fitView
      >
        <Controls />
        <MiniMap />
        <Background variant="dots" gap={12} size={1} />
      </ReactFlow>
    </div>
  );
};

export default MissionEditor;

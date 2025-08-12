import React, { useState, useCallback, useEffect } from 'react';
import ReactFlow, {
  addEdge,
  applyEdgeChanges,
  applyNodeChanges,
  Controls,
  MiniMap,
  Background,
} from 'reactflow';
import 'reactflow/dist/style.css';
import NavigateToPoseNode from '../components/NavigateToPoseNode';
import { get as apiGet, post as apiPost } from '../services/apiService';

const nodeTypes = {
  navigateToPose: NavigateToPoseNode,
};

const initialNodes = [
  { id: '1', type: 'input', position: { x: 250, y: 5 }, data: { label: 'Start Mission' } },
];

let id = 2;
const getId = () => `${id++}`;

const MissionEditor = () => {
  const [nodes, setNodes] = useState(initialNodes);
  const [edges, setEdges] = useState([]);
  const [missionName, setMissionName] = useState('');
  const [mapId, setMapId] = useState('');
  const [maps, setMaps] = useState([]);
  const [status, setStatus] = useState({ message: '', type: '' }); // { message, type: 'success' | 'error' | 'info' }
  const [missions, setMissions] = useState([]);
  const [selectedMission, setSelectedMission] = useState('');
  const [isRunning, setIsRunning] = useState(false);

  useEffect(() => {
    if (status.message) {
      const timer = setTimeout(() => {
        setStatus({ message: '', type: '' });
      }, 5000); // Clear status after 5 seconds
      return () => clearTimeout(timer);
    }
  }, [status]);

  const onNodeDataChange = (nodeId, newData) => {
    setNodes((nds) =>
      nds.map((node) => {
        if (node.id === nodeId) {
          return {
            ...node,
            data: { ...node.data, ...newData },
          };
        }
        return node;
      })
    );
  };

  const parseMissionDefinition = (definition) => {
    const newNodes = [];
    const newEdges = [];
    let lastNodeId = '1';
    newNodes.push({ id: '1', type: 'input', position: { x: 250, y: 5 }, data: { label: 'Start Mission' } });

    if (definition.root && definition.root.children) {
      definition.root.children.forEach((child, index) => {
        const newNodeId = getId();
        const newNode = {
          id: newNodeId,
          type: child.type,
          position: { x: 250, y: 100 + index * 150 },
          data: {
            pointId: child.params?.point_id || '',
            onChange: onNodeDataChange,
            mapId: mapId,
          },
        };
        newNodes.push(newNode);
        newEdges.push({ id: `e${lastNodeId}-${newNodeId}`, source: lastNodeId, target: newNodeId });
        lastNodeId = newNodeId;
      });
    }
    return { nodes: newNodes, edges: newEdges };
  };

  useEffect(() => {
    const fetchMaps = async () => {
      try {
        const mapsData = await apiGet('/maps');
        setMaps(mapsData);
        if (mapsData.length > 0) {
          setMapId(mapsData[0]._id);
        }
      } catch (error) {
        console.error("Failed to fetch maps:", error);
        setStatus({ message: 'Failed to fetch maps', type: 'error' });
      }
    };
    fetchMaps();
  }, []);

  useEffect(() => {
    const fetchMissions = async () => {
      if (mapId) {
        try {
          const missionsData = await apiGet(`/missions?map_id=${mapId}`);
          setMissions(missionsData);
          setSelectedMission('');
        } catch (error) {
          console.error('Failed to fetch missions:', error);
          setStatus({ message: 'Failed to fetch missions for this map', type: 'error' });
        }
      }
    };
    fetchMissions();
  }, [mapId]);

  const onClear = () => {
    setNodes(initialNodes);
    setEdges([]);
    setMissionName('');
    setSelectedMission('');
    setStatus({ message: 'Editor cleared', type: 'info' });
  };

  const onLoad = async () => {
    if (!selectedMission) {
      setStatus({ message: 'Please select a mission to load.', type: 'info' });
      return;
    }
    setStatus({ message: `Loading mission...`, type: 'info' });
    try {
      const missionData = await apiGet(`/missions/${selectedMission}`);
      const { nodes: loadedNodes, edges: loadedEdges } = parseMissionDefinition(missionData.definition);
      
      setNodes(loadedNodes);
      setEdges(loadedEdges);
      setMissionName(missionData.name);
      setStatus({ message: 'Mission loaded successfully!', type: 'success' });
    } catch (error) {
      console.error('Failed to load mission:', error);
      setStatus({ message: `Error loading mission: ${error.message}`, type: 'error' });
    }
  };

  const buildMissionDefinition = () => {
    const startNode = nodes.find(n => n.type === 'input');
    if (!startNode) throw new Error("Mission must have a start node.");

    const sequence = [];
    let currentNodeId = startNode.id;
    while (currentNodeId) {
      const edge = edges.find(e => e.source === currentNodeId);
      if (!edge) break;
      const nextNode = nodes.find(n => n.id === edge.target);
      if (!nextNode) break;
      sequence.push(nextNode);
      currentNodeId = nextNode.id;
    }

    if (sequence.length === 0) throw new Error("Mission is empty.");

    return {
      root: {
        type: 'Sequence',
        children: sequence.map(node => ({
          type: node.type,
          params: { point_id: node.data.pointId },
        })),
      },
    };
  };

  const onSave = async () => {
    if (!missionName) {
      setStatus({ message: 'Please enter a mission name.', type: 'info' });
      return;
    }
    setStatus({ message: 'Saving...', type: 'info' });
    try {
      const missionDefinition = buildMissionDefinition();
      const missionData = {
        name: missionName,
        map_id: mapId,
        loop: false,
        definition: missionDefinition,
      };
      await apiPost('/missions', missionData);
      setStatus({ message: 'Mission saved successfully!', type: 'success' });
      // Refresh missions list
      const missionsData = await apiGet(`/missions?map_id=${mapId}`);
      setMissions(missionsData);
    } catch (error) {
      console.error('Failed to save mission:', error);
      setStatus({ message: `Error saving mission: ${error.message}`, type: 'error' });
    }
  };

  const onRun = async () => {
    if (!selectedMission) {
      setStatus({ message: 'Please select a mission to run.', type: 'info' });
      return;
    }
    setIsRunning(true);
    setStatus({ message: `Running mission...`, type: 'info' });
    try {
      await apiPost(`/missions/${selectedMission}/run`);
      setStatus({ message: 'Mission started successfully!', type: 'success' });
    } catch (error) {
      console.error('Failed to run mission:', error);
      setStatus({ message: `Error running mission: ${error.message}`, type: 'error' });
    } finally {
      setIsRunning(false);
    }
  };

  const onNodesChange = useCallback((changes) => setNodes((nds) => applyNodeChanges(changes, nds)), [setNodes]);
  const onEdgesChange = useCallback((changes) => setEdges((eds) => applyEdgeChanges(changes, eds)), [setEdges]);
  const onConnect = useCallback((connection) => setEdges((eds) => addEdge(connection, eds)), [setEdges]);

  const onAddNode = () => {
    const newNode = {
      id: getId(),
      type: 'navigateToPose',
      position: {
        x: Math.random() * 400,
        y: Math.random() * 400,
      },
      data: { pointId: '', onChange: onNodeDataChange, mapId: mapId },
    };
    setNodes((nds) => nds.concat(newNode));
  };

  const getStatusStyle = () => {
    switch (status.type) {
      case 'success':
        return { color: 'green', fontWeight: 'bold' };
      case 'error':
        return { color: 'red', fontWeight: 'bold' };
      default:
        return { color: 'blue' };
    }
  };

  return (
    <div>
      <div style={{ marginBottom: '10px', display: 'flex', alignItems: 'center', gap: '10px', flexWrap: 'wrap' }}>
        <input
          type="text"
          placeholder="Mission Name"
          value={missionName}
          onChange={(e) => setMissionName(e.target.value)}
          style={{ padding: '8px' }}
        />
        <select
          value={mapId}
          onChange={(e) => setMapId(e.target.value)}
          style={{ padding: '8px' }}
        >
          {maps.map((map) => (
            <option key={map._id} value={map._id}>
              {map.name}
            </option>
          ))}
        </select>
        <select
          value={selectedMission}
          onChange={(e) => setSelectedMission(e.target.value)}
          style={{ padding: '8px' }}
        >
          <option value="">Select a mission to load</option>
          {missions.map((mission) => (
            <option key={mission._id} value={mission._id}>
              {mission.name}
            </option>
          ))}
        </select>
        <button onClick={onLoad} style={{ padding: '10px', background: '#ffc107', color: 'black', border: 'none', borderRadius: '5px' }}>
          Load Mission
        </button>
        <button onClick={onAddNode} style={{ padding: '10px', background: '#007bff', color: 'white', border: 'none', borderRadius: '5px' }}>
          Add NavigateToPose
        </button>
        <button onClick={onSave} style={{ padding: '10px', background: '#28a745', color: 'white', border: 'none', borderRadius: '5px' }}>
          Save Mission
        </button>
        <button 
          onClick={onRun} 
          style={{
            padding: '10px', 
            background: '#17a2b8', 
            color: 'white', 
            border: 'none', 
            borderRadius: '5px',
            cursor: isRunning || !selectedMission ? 'not-allowed' : 'pointer',
            opacity: isRunning || !selectedMission ? 0.6 : 1
          }}
          disabled={isRunning || !selectedMission}
        >
          {isRunning ? 'Running...' : 'Run Mission'}
        </button>
        <button onClick={onClear} style={{ padding: '10px', background: '#dc3545', color: 'white', border: 'none', borderRadius: '5px' }}>
          Clear
        </button>
        {status.message && <div style={{ ...getStatusStyle(), marginLeft: '10px', alignSelf: 'center' }}>{status.message}</div>}
      </div>
      <div style={{ height: 'calc(100vh - 220px)', border: '1px solid #ccc', background: '#f8f8f8' }}>
        <ReactFlow
          nodes={nodes}
          edges={edges}
          onNodesChange={onNodesChange}
          onEdgesChange={onEdgesChange}
          onConnect={onConnect}
          nodeTypes={nodeTypes}
          fitView
        >
          <Controls />
          <MiniMap />
          <Background variant="dots" gap={12} size={1} />
        </ReactFlow>
      </div>
    </div>
  );
};

export default MissionEditor;

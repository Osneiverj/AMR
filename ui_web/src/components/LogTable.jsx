import { useEffect, useState } from 'react';
import ros from '../services/rosService';

export default function LogTable() {
  const [logs, setLogs] = useState([]);

  useEffect(() => {
    const sub = ros.subscribe('/rosout', 'rosgraph_msgs/Log', msg =>
      setLogs(l => [
        { level: msg.level, text: msg.msg, time: new Date(msg.header.stamp.secs * 1000) },
        ...l.slice(0, 49)
      ])
    );
    return () => sub.unsubscribe();
  }, []);

  return (
    <div className="bg-white rounded-lg shadow overflow-auto max-h-64">
      <table className="w-full text-sm">
        <thead>
          <tr className="bg-gray-100 sticky top-0">
            <th className="px-2">Time</th>
            <th className="px-2">Level</th>
            <th className="px-2">Message</th>
          </tr>
        </thead>
        <tbody>
          {logs.map((l, i) => (
            <tr key={i} className="border-t">
              <td className="px-2 whitespace-nowrap">{l.time.toLocaleTimeString()}</td>
              <td className="px-2">{l.level}</td>
              <td className="px-2">{l.text}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}

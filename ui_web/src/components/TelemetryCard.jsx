import { useEffect, useState } from 'react';
import ros from '../services/rosService';

export default function TelemetryCard() {
  const [battery, setBattery] = useState(0);

  useEffect(() => {
    const sub = ros.subscribe('/battery_state', 'sensor_msgs/BatteryState', msg =>
      setBattery(Math.round(msg.percentage * 100))
    );
    return () => sub.unsubscribe();
  }, []);

  return (
    <div className="bg-white p-4 rounded-lg shadow flex items-center gap-4">
      <span className="text-4xl">🔋</span>
      <div>
        <h3 className="text-sm font-medium text-gray-500">Battery</h3>
        <p className="text-xl font-semibold">{battery}%</p>
      </div>
    </div>
  );
}

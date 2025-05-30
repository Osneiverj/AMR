import { useRef, useEffect } from 'react';
import ros from '../services/rosService';

export default function Joystick() {
  const ws = useRef(null);

  useEffect(() => {
    const pub = ros.advertise('/cmd_vel', 'geometry_msgs/Twist');

    const handleMove = e => {
      // TODO: manejar joystick real; por ahora teclas
      const twist = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
      if (e.key === 'ArrowUp') twist.linear.x = 0.2;
      if (e.key === 'ArrowDown') twist.linear.x = -0.2;
      if (e.key === 'ArrowLeft') twist.angular.z = 0.5;
      if (e.key === 'ArrowRight') twist.angular.z = -0.5;
      pub.publish(twist);
    };

    window.addEventListener('keydown', handleMove);
    return () => {
      window.removeEventListener('keydown', handleMove);
      pub.unadvertise();
    };
  }, []);

  return (
    <div className="bg-white p-4 rounded-lg shadow text-center">
      <p className="text-gray-600">Use arrow keys to tele-op</p>
    </div>
  );
}

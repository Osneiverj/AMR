import { useRef } from 'react';
import { Joystick } from 'react-joystick-component';
import ros from '../services/rosService';
import { DEFAULT_LINEAR, DEFAULT_ANGULAR } from '../utils/constants';

export default function Jog() {
  const cmdVel = useRef(
    ros.advertise('/cmd_vel', 'geometry_msgs/Twist')
  ).current;

  const handleMove = (stick) => {
    // stick.x/y están en rango -100..100
    const lin = (stick.y / 100) * DEFAULT_LINEAR;   // adelante ↑  / atrás ↓ --- ↑ = +x   ↓ = -x
    const ang =  -(stick.x / 100) * DEFAULT_ANGULAR;  // der  →      / izq  ← --- → = +z   ← = -z
    cmdVel.publish({
      linear:  { x: lin, y: 0, z: 0 },
      angular: { x: 0,   y: 0, z: ang }
    });
  };

  const stop = () => cmdVel.publish({ linear: {}, angular: {} });

  return (
    <div className="bg-white p-4 rounded-lg shadow flex flex-col items-center">
      <h3 className="font-medium mb-2">Manual Jog</h3>
      <Joystick
        size={120}
        baseColor="#e5e7eb"
        stickColor="#1d4ed8"
        throttle={100}
        move={handleMove}
        stop={stop}
      />
    </div>
  );
}

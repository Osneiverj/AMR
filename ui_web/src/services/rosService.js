import ROSLIB from 'roslib';

class RosService {
  constructor() {
    this.ros = new ROSLIB.Ros({ url: __ROSBRIDGE_URL__ });
    this.ros.on('connection', () => console.log('[ROS] connected'));
    this.ros.on('error', err => console.error('[ROS] error', err));
    this.ros.on('close', () => console.warn('[ROS] disconnected'));
  }

  subscribe(topic, type, cb) {
    const listener = new ROSLIB.Topic({ ros: this.ros, name: topic, messageType: type });
    listener.subscribe(cb);
    return listener;
  }

  advertise(topic, type) {
    return new ROSLIB.Topic({ ros: this.ros, name: topic, messageType: type });
  }

  callService(name, type, request = {}) {
    return new Promise((res, rej) => {
      const srv = new ROSLIB.Service({ ros: this.ros, name, serviceType: type });
      srv.callService(new ROSLIB.ServiceRequest(request), res, rej);
    });
  }
}

const ros = new RosService();
export default ros;

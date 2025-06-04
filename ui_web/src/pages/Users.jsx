import { useState } from 'react';
import { useAuth } from '../AuthContext';
import { register, changePassword } from '../services/authService';

export default function Users() {
  const { token } = useAuth();
  const [username, setUsername] = useState('');
  const [password, setPassword] = useState('');
  const [role, setRole] = useState('user');
  const [newPass, setNewPass] = useState('');
  const [msg, setMsg] = useState('');

  const createUser = async e => {
    e.preventDefault();
    try {
      await register({ username, password, role }, token);
      setMsg('User created');
    } catch {
      setMsg('Error creating user');
    }
  };

  const change = async e => {
    e.preventDefault();
    try {
      await changePassword(newPass, token);
      setMsg('Password updated');
    } catch {
      setMsg('Error updating password');
    }
  };

  return (
    <div className="space-y-8 max-w-md">
      <form onSubmit={createUser} className="space-y-2 p-4 bg-white rounded shadow">
        <h2 className="font-semibold">Create User</h2>
        <input className="border p-2 w-full" placeholder="Username" value={username} onChange={e => setUsername(e.target.value)} />
        <input type="password" className="border p-2 w-full" placeholder="Password" value={password} onChange={e => setPassword(e.target.value)} />
        <select className="border p-2 w-full" value={role} onChange={e => setRole(e.target.value)}>
          <option value="user">user</option>
          <option value="admin">admin</option>
        </select>
        <button className="bg-blue-500 text-white px-4 py-2 rounded w-full">Create</button>
      </form>

      <form onSubmit={change} className="space-y-2 p-4 bg-white rounded shadow">
        <h2 className="font-semibold">Change My Password</h2>
        <input type="password" className="border p-2 w-full" placeholder="New Password" value={newPass} onChange={e => setNewPass(e.target.value)} />
        <button className="bg-blue-500 text-white px-4 py-2 rounded w-full">Change</button>
      </form>

      {msg && <p>{msg}</p>}
    </div>
  );
}

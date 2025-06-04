import { Routes, Route, NavLink } from 'react-router-dom';
import Dashboard from './pages/Dashboard';
import Settings from './pages/Settings';
import Users from './pages/Users';

import Login from './pages/Login';
import ProtectedRoute from './ProtectedRoute';
import { useAuth } from './AuthContext';

export default function App() {
  const linkStyle = 'px-4 py-2 hover:text-blue-600';
  const active = ({ isActive }) =>
    isActive ? `${linkStyle} text-blue-600 font-semibold` : linkStyle;
  const { token, logout } = useAuth();

  return (
    <div className="min-h-screen flex flex-col">
      <header className="bg-white shadow">
        <nav className="container mx-auto flex gap-4">
          <NavLink to="/" end className={active}>
            Dashboard
          </NavLink>
          <NavLink to="/settings" className={active}>
            Settings
          </NavLink>
          <NavLink to="/users" className={active}>
            Users
          </NavLink>

          {token && (
            <button onClick={logout} className="ml-auto px-4 py-2 text-sm">
              Logout
            </button>
          )}
        </nav>
      </header>

      <main className="flex-1 container mx-auto p-4">
        <Routes>
          <Route path="/login" element={<Login />} />
          <Route path="/" element={<ProtectedRoute><Dashboard /></ProtectedRoute>} />
          <Route path="/settings" element={<ProtectedRoute><Settings /></ProtectedRoute>} />
          <Route path="/users" element={<ProtectedRoute><Users /></ProtectedRoute>} />

        </Routes>
      </main>
    </div>
  );
}

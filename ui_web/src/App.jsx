import { Routes, Route, NavLink } from 'react-router-dom';
import { Dashboard, Settings, Users, Maps, Points, Missions, Login } from './pages';
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

      <div className="flex flex-1">
        <aside className="w-48 bg-gray-100 p-4">
          <ul className="space-y-2">
            <li>
              <NavLink to="/maps" className={active}>
                Mapas
              </NavLink>
            </li>
            <li>
              <NavLink to="/points" className={active}>
                Puntos
              </NavLink>
            </li>
            <li>
              <NavLink to="/missions" className={active}>
                Misiones
              </NavLink>
            </li>
          </ul>
        </aside>
        <main className="flex-1 container mx-auto p-4">
          <Routes>
            <Route path="/login" element={<Login />} />
            <Route path="/" element={<ProtectedRoute><Dashboard /></ProtectedRoute>} />
            <Route path="/settings" element={<ProtectedRoute><Settings /></ProtectedRoute>} />
            <Route path="/users" element={<ProtectedRoute><Users /></ProtectedRoute>} />
            <Route path="/maps" element={<ProtectedRoute><Maps /></ProtectedRoute>} />
            <Route path="/points" element={<ProtectedRoute><Points /></ProtectedRoute>} />
            <Route path="/missions" element={<ProtectedRoute><Missions /></ProtectedRoute>} />
          </Routes>
        </main>
      </div>
    </div>
  );
}

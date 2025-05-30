import { Routes, Route, NavLink } from 'react-router-dom';
import Dashboard from './pages/Dashboard';
import Settings from './pages/Settings';

export default function App() {
  const linkStyle = 'px-4 py-2 hover:text-blue-600';
  const active = ({ isActive }) =>
    isActive ? `${linkStyle} text-blue-600 font-semibold` : linkStyle;

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
        </nav>
      </header>

      <main className="flex-1 container mx-auto p-4">
        <Routes>
          <Route path="/" element={<Dashboard />} />
          <Route path="/settings" element={<Settings />} />
        </Routes>
      </main>
    </div>
  );
}

import { createContext, useContext, useState } from 'react';
import { login as apiLogin } from './services/authService';

const AuthContext = createContext(null);

export function AuthProvider({ children }) {
  const [token, setToken] = useState(() => localStorage.getItem('token'));

  const login = async (username, password) => {
    const data = await apiLogin(username, password);
    setToken(data.access_token);
    localStorage.setItem('token', data.access_token);
  };

  const logout = () => {
    setToken(null);
    localStorage.removeItem('token');
  };

  return (
    <AuthContext.Provider value={{ token, login, logout }}>
      {children}
    </AuthContext.Provider>
  );
}

export const useAuth = () => useContext(AuthContext);
